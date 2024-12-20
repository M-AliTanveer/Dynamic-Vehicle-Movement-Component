// Fill out your copyright notice in the Description page of Project Settings.


#include "VehicleComponent/DynamicVehicleMovementComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Components/SkinnedMeshComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Engine/SkeletalMesh.h"
#include "DrawDebugHelpers.h"
#include "DisplayDebugHelpers.h"
#include "DisplayDebugHelpers.h"
#include "Engine/Engine.h"
#include "GameFramework/Pawn.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "VehicleComponent/AnimInstance/DynamicVehicleAnimationInstance.h"
#include "ChaosVehicleManager.h"
#include "Wheels/DynamicWheel/DynamicWheel.h"
#include "SuspensionUtility.h"
#include "SteeringUtility.h"
#include "TransmissionUtility.h"
#include "Chaos/ChaosEngineInterface.h"
#include "Chaos/PBDSuspensionConstraintData.h"
#include "Chaos/DebugDrawQueue.h"
#include "UObject/UE5MainStreamObjectVersion.h"
#include "Kismet/KismetMathLibrary.h"
#include "PhysicsProxy/SuspensionConstraintProxy.h"
#include "PBDRigidsSolver.h"
#include "Kismet/GameplayStatics.h"
#include "Components/SpotLightComponent.h"
#include "TimerManager.h"
#include "Components/LightComponent.h"


#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
#include "CanvasItem.h"
#include "Engine/Canvas.h"
#endif
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"

#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_DISABLE_OPTIMIZATION
#endif

DECLARE_STATS_GROUP(TEXT("ChaosVehicle"), STATGROUP_ChaosVehicle, STATGROUP_Advanced);

DECLARE_CYCLE_STAT(TEXT("Vehicle:SuspensionRaycasts"), STAT_ChaosVehicle_SuspensionRaycasts, STATGROUP_ChaosVehicle);
DECLARE_CYCLE_STAT(TEXT("Vehicle:SuspensionOverlapTest"), STAT_ChaosVehicle_SuspensionOverlapTest, STATGROUP_ChaosVehicle);
DECLARE_CYCLE_STAT(TEXT("Vehicle:SuspensionTraces"), STAT_ChaosVehicle_SuspensionTraces, STATGROUP_ChaosVehicle);
DECLARE_CYCLE_STAT(TEXT("Vehicle:TickVehicle"), STAT_ChaosVehicle_TickVehicle, STATGROUP_ChaosVehicle);
DECLARE_CYCLE_STAT(TEXT("Vehicle:UpdateSimulation"), STAT_ChaosVehicle_UpdateSimulation, STATGROUP_ChaosVehicle);


FDynamicWheeledVehicleDebugParams DynWheeledVehicleDebugParams;
FVehicleDebugParams DynVehicleDebugParams;


FString FDynamicWheelStatus::ToString() const
{
	return FString::Printf(TEXT("bInContact:%s ContactPoint:%s PhysMaterial:%s NormSuspensionLength:%f SpringForce:%f SlipAngle:%f bIsSlipping:%s SlipMagnitude:%f bIsSkidding:%s SkidMagnitude:%f SkidNormal:%s DriveTorque:%f BrakeTorque:%f ABSActive:%s"),
		bInContact == true ? TEXT("True") : TEXT("False"),
		*ContactPoint.ToString(),
		PhysMaterial.IsValid() ? *PhysMaterial->GetName() : TEXT("None"),
		NormalizedSuspensionLength,
		SpringForce,
		SlipAngle,
		bIsSlipping == true ? TEXT("True") : TEXT("False"),
		SlipMagnitude,
		bIsSkidding == true ? TEXT("True") : TEXT("False"),
		SkidMagnitude,
		*SkidNormal.ToString(),
		DriveTorque,
		BrakeTorque,
		bABSActivated == true ? TEXT("True") : TEXT("False"));
}

void FDynamicWheelState::CaptureState(int WheelIdx, const FVector& WheelOffset, const FBodyInstance* TargetInstance)
{
	check(TargetInstance);
	const FTransform WorldTransform = TargetInstance->GetUnrealWorldTransform();
	WheelLocalLocation[WheelIdx] = WheelOffset;
	WheelWorldLocation[WheelIdx] = WorldTransform.TransformPosition(WheelOffset);
	WorldWheelVelocity[WheelIdx] = TargetInstance->GetUnrealWorldVelocityAtPoint(WheelWorldLocation[WheelIdx]);
	LocalWheelVelocity[WheelIdx] = WorldTransform.InverseTransformVector(WorldWheelVelocity[WheelIdx]);
}

void FDynamicWheelState::CaptureState(int WheelIdx, const FVector& WheelOffset, const Chaos::FRigidBodyHandle_Internal* Handle)
{
	check(Handle);
	const FTransform WorldTransform(Handle->R(), Handle->X());
	WheelLocalLocation[WheelIdx] = WheelOffset;
	WheelWorldLocation[WheelIdx] = WorldTransform.TransformPosition(WheelOffset);
	WorldWheelVelocity[WheelIdx] = GetVelocityAtPoint(Handle, WheelWorldLocation[WheelIdx]);
	LocalWheelVelocity[WheelIdx] = WorldTransform.InverseTransformVector(WorldWheelVelocity[WheelIdx]);
}

void FDynamicWheelState::CaptureState(int WheelIdx, const FVector& WheelOffset, const Chaos::FRigidBodyHandle_Internal* VehicleHandle, const FVector& ContactPoint, const Chaos::FRigidBodyHandle_Internal* SurfaceHandle)
{
	check(VehicleHandle);

	FVector SurfaceVelocity = FVector::ZeroVector;
	if (SurfaceHandle)
	{
		SurfaceVelocity = GetVelocityAtPoint(SurfaceHandle, ContactPoint);
	}

	const FTransform WorldTransform(VehicleHandle->R(), VehicleHandle->X());
	WheelLocalLocation[WheelIdx] = WheelOffset;
	WheelWorldLocation[WheelIdx] = WorldTransform.TransformPosition(WheelOffset);
	WorldWheelVelocity[WheelIdx] = GetVelocityAtPoint(VehicleHandle, WheelWorldLocation[WheelIdx]) - SurfaceVelocity;
	LocalWheelVelocity[WheelIdx] = WorldTransform.InverseTransformVector(WorldWheelVelocity[WheelIdx]);
}

FVector FDynamicWheelState::GetVelocityAtPoint(const Chaos::FRigidBodyHandle_Internal* Rigid, const FVector& InPoint)
{
	if (Rigid)
	{
		const Chaos::FVec3 COM = Rigid ? Chaos::FParticleUtilitiesGT::GetCoMWorldPosition(Rigid) : (Chaos::FVec3)Chaos::FParticleUtilitiesGT::GetActorWorldTransform(Rigid).GetTranslation();
		const Chaos::FVec3 Diff = InPoint - COM;
		return Rigid->V() - Chaos::FVec3::CrossProduct(Diff, Rigid->W());
	}
	else
	{
		return FVector::ZeroVector;
	}
}

bool UDynamicVehicleSimulation::CanSimulate() const
{
	if (UChaosVehicleSimulation::CanSimulate() == false)
	{
		return false;
	}

	return (PVehicle && PVehicle.IsValid()
		&& PVehicle->Engine.Num() == PVehicle->Transmission.Num()
		&& PVehicle->Wheels.Num() == PVehicle->Suspension.Num());
}

void UDynamicVehicleSimulation::TickVehicle(UWorld* WorldIn, float DeltaTime, const FChaosVehicleAsyncInput& InputData, FChaosVehicleAsyncOutput& OutputData, Chaos::FRigidBodyHandle_Internal* Handle)
{
	SCOPE_CYCLE_COUNTER(STAT_ChaosVehicle_TickVehicle);

	UChaosVehicleSimulation::TickVehicle(WorldIn, DeltaTime, InputData, OutputData, Handle);
}

void UDynamicVehicleSimulation::UpdateState(float DeltaTime, const FChaosVehicleAsyncInput& InputData, Chaos::FRigidBodyHandle_Internal* Handle)
{
	UChaosVehicleSimulation::UpdateState(DeltaTime, InputData, Handle);

	if (CanSimulate() && Handle)
	{
		// sanity check that everything is setup ok
		ensure(PVehicle->Wheels.Num() == PVehicle->Suspension.Num());
		ensure(WheelState.LocalWheelVelocity.Num() == PVehicle->Wheels.Num());
		ensure(WheelState.WheelWorldLocation.Num() == PVehicle->Wheels.Num());
		ensure(WheelState.WorldWheelVelocity.Num() == PVehicle->Wheels.Num());

		///////////////////////////////////////////////////////////////////////
		// Cache useful state so we are not re-calculating the same data
		for (int WheelIdx = 0; WheelIdx < PVehicle->Suspension.Num(); WheelIdx++)
		{
			bool bCaptured = false;

			// #TODO: This is not threadsafe - need to rethink how to get the rigidbody that is hit by the raycast
			//const FHitResult& HitResult = Wheels[WheelIdx]->HitResult;
			//if (HitResult.Component.IsValid() && HitResult.Component->GetBodyInstance())
			//{
			//	if (const FPhysicsActorHandle& SurfaceHandle = HitResult.Component->GetBodyInstance()->GetPhysicsActorHandle())
			//	{
			//		// we are being called from the physics thread
			//		if (Chaos::FRigidBodyHandle_Internal* SurfaceBody = SurfaceHandle->GetPhysicsThreadAPI())
			//		{
			//			if (SurfaceBody->CanTreatAsKinematic())
			//			{
			//				FVector Point = HitResult.ImpactPoint;
			//				WheelState.CaptureState(WheelIdx, PVehicle->Suspension[WheelIdx].GetLocalRestingPosition(), Handle, Point, SurfaceBody);
			//				bCaptured = true;
			//			}
			//		}
			//	}
			//}

			if (!bCaptured)
			{
				WheelState.CaptureState(WheelIdx, PVehicle->Suspension[WheelIdx].GetLocalRestingPosition(), Handle);
			}
		}
		///////////////////////////////////////////////////////////////////////
		// Suspension Raycast

		for (int WheelIdx = 0; WheelIdx < PVehicle->Suspension.Num(); WheelIdx++)
		{
			auto& PSuspension = PVehicle->Suspension[WheelIdx];
			auto& PWheel = PVehicle->Wheels[WheelIdx];
			PSuspension.UpdateWorldRaycastLocation(VehicleState.VehicleWorldTransform, PWheel.GetEffectiveRadius(), WheelState.Trace[WheelIdx]);
		}

		if (!DynWheeledVehicleDebugParams.DisableSuspensionForces && PVehicle->bSuspensionEnabled)
		{
			PerformSuspensionTraces(WheelState.Trace, InputData.PhysicsInputs.TraceParams, InputData.PhysicsInputs.TraceCollisionResponse, InputData.PhysicsInputs.WheelTraceParams);
		}

		//////////////////////////////////////////////////////////////////////////
		// Wheel and Vehicle in air state

		VehicleState.bVehicleInAir = true;
		VehicleState.NumWheelsOnGround = 0;
		for (int WheelIdx = 0; WheelIdx < PVehicle->Wheels.Num(); WheelIdx++)
		{
			// tell systems who care that wheel is touching the ground
			PVehicle->Wheels[WheelIdx].SetOnGround(WheelState.TraceResult[WheelIdx].bBlockingHit);

			// only requires one wheel to be on the ground for the vehicle to be NOT in the air
			if (PVehicle->Wheels[WheelIdx].InContact())
			{
				VehicleState.bVehicleInAir = false;
				VehicleState.NumWheelsOnGround++;
			}
		}
		VehicleState.bAllWheelsOnGround = (VehicleState.NumWheelsOnGround == PVehicle->Wheels.Num());
	}
}

void UDynamicVehicleSimulation::UpdateSimulation(float DeltaTime, const FChaosVehicleAsyncInput& InputData, Chaos::FRigidBodyHandle_Internal* Handle)
{
	SCOPE_CYCLE_COUNTER(STAT_ChaosVehicle_UpdateSimulation);

	// Inherit common vehicle simulation stages ApplyAerodynamics, ApplyTorqueControl, etc
	UChaosVehicleSimulation::UpdateSimulation(DeltaTime, InputData, Handle);

	if (CanSimulate() && Handle)
	{
		///////////////////////////////////////////////////////////////////////
		// Engine/Transmission
		if (!DynWheeledVehicleDebugParams.DisableSuspensionForces && PVehicle->bMechanicalSimEnabled)
		{
			ProcessMechanicalSimulation(DeltaTime);
		}

		///////////////////////////////////////////////////////////////////////
		// Suspension

		if (!DynWheeledVehicleDebugParams.DisableSuspensionForces && PVehicle->bSuspensionEnabled)
		{
			ApplySuspensionForces(DeltaTime, InputData.PhysicsInputs.WheelTraceParams);
		}

		///////////////////////////////////////////////////////////////////////
		// Steering

		ProcessSteering(InputData.PhysicsInputs.NetworkInputs.VehicleInputs);

		///////////////////////////////////////////////////////////////////////
		// Wheel Friction

		if (!DynWheeledVehicleDebugParams.DisableFrictionForces && PVehicle->bWheelFrictionEnabled)
		{
			ApplyWheelFrictionForces(DeltaTime);
		}

#if 0
		if (PerformanceMeasure.IsEnabled())
		{
			PerformanceMeasure.Update(DeltaTime, VehicleState.VehicleWorldTransform.GetLocation(), VehicleState.ForwardSpeed);
		}
#endif
	}
}

bool UDynamicVehicleSimulation::ContainsTraces(const FBox& Box, const TArray<Chaos::FSuspensionTrace>& SuspensionTrace)
{
	const Chaos::FAABB3 Aabb(Box.Min, Box.Max);

	for (int WheelIdx = 0; WheelIdx < SuspensionTrace.Num(); WheelIdx++)
	{
		if (!Aabb.Contains(SuspensionTrace[WheelIdx].Start) || !Aabb.Contains(SuspensionTrace[WheelIdx].End))
		{
			return false;
		}
	}

	return true;
}

void UDynamicVehicleSimulation::PerformSuspensionTraces(const TArray<Chaos::FSuspensionTrace>& SuspensionTrace, FCollisionQueryParams& TraceParams, FCollisionResponseContainer& CollisionResponse, TArray<FWheelTraceParams>& WheelTraceParams)
{
	SCOPE_CYCLE_COUNTER(STAT_ChaosVehicle_SuspensionRaycasts);

	ECollisionChannel SpringCollisionChannel = ECollisionChannel::ECC_WorldDynamic;
	FCollisionResponseParams ResponseParams;
	ResponseParams.CollisionResponse = CollisionResponse;

	// batching is about 0.5ms (25%) faster when there's 100 vehicles on a flat terrain
	if (DynVehicleDebugParams.BatchQueries)
	{
		if (!DynVehicleDebugParams.CacheTraceOverlap || !ContainsTraces(QueryBox, SuspensionTrace))
		{
			SCOPE_CYCLE_COUNTER(STAT_ChaosVehicle_SuspensionOverlapTest);

			bOverlapHit = false;
			OverlapResults.Empty();
			QueryBox.Init();

			//FBox QueryBox;
			for (int WheelIdx = 0; WheelIdx < SuspensionTrace.Num(); WheelIdx++)
			{
				const FVector& TraceStart = SuspensionTrace[WheelIdx].Start;
				const FVector& TraceEnd = SuspensionTrace[WheelIdx].End;

				if (WheelIdx == 0)
				{
					QueryBox = FBox(TraceStart, TraceEnd);
				}
				else
				{
					QueryBox.Min = QueryBox.Min.ComponentMin(TraceStart);
					QueryBox.Min = QueryBox.Min.ComponentMin(TraceEnd);
					QueryBox.Max = QueryBox.Max.ComponentMax(TraceStart);
					QueryBox.Max = QueryBox.Max.ComponentMax(TraceEnd);
				}
			}
			QueryBox = QueryBox.ExpandBy(FVector(DynWheeledVehicleDebugParams.OverlapTestExpansionXY, DynWheeledVehicleDebugParams.OverlapTestExpansionXY, DynWheeledVehicleDebugParams.OverlapTestExpansionZ));
			FCollisionShape CollisionBox;
			CollisionBox.SetBox((FVector3f)QueryBox.GetExtent());

			bOverlapHit = World->OverlapMultiByChannel(OverlapResults, QueryBox.GetCenter(), FQuat::Identity, SpringCollisionChannel, CollisionBox, TraceParams, ResponseParams);
		}

#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
		if (DynWheeledVehicleDebugParams.ShowBatchQueryExtents)
		{
			Chaos::FDebugDrawQueue::GetInstance().DrawDebugBox(QueryBox.GetCenter(), QueryBox.GetExtent(), FQuat::Identity, FColor::Yellow, false, -1.0f, 0, 2.0f);

			// draw all corresponding results bounding boxes
			for (FOverlapResult OverlapResult : OverlapResults)
			{
				if (OverlapResult.bBlockingHit)
				{
					const FBoxSphereBounds Bounds = OverlapResult.Component->CalcBounds(OverlapResult.Component->GetComponentTransform());
					Chaos::FDebugDrawQueue::GetInstance().DrawDebugBox(Bounds.GetBox().GetCenter(), Bounds.GetBox().GetExtent(), FQuat::Identity, FColor::Purple, false, -1.0f, 0, 2.0f);
				}
			}
		}
#endif

		SCOPE_CYCLE_COUNTER(STAT_ChaosVehicle_SuspensionTraces);
		for (int32 WheelIdx = 0; WheelIdx < SuspensionTrace.Num(); ++WheelIdx)
		{
			FHitResult& HitResult = WheelState.TraceResult[WheelIdx];
			HitResult = FHitResult();

			if (bOverlapHit)
			{
				const FVector& TraceStart = SuspensionTrace[WheelIdx].Start;
				const FVector& TraceEnd = SuspensionTrace[WheelIdx].End;
				TraceParams.bTraceComplex = (WheelTraceParams[WheelIdx].SweepType == ESweepType::ComplexSweep);

				if (DynWheeledVehicleDebugParams.TraceTypeOverride > 0)
				{
					TraceParams.bTraceComplex = DynWheeledVehicleDebugParams.TraceTypeOverride == 2;
				}

				FVector TraceVector(TraceStart - TraceEnd); // reversed
				FVector TraceNormal = TraceVector.GetSafeNormal();

				// Test each overlapped object for a hit result
				for (FOverlapResult OverlapResult : OverlapResults)
				{
					if (!OverlapResult.bBlockingHit)
						continue;

					FHitResult ComponentHit;

					switch (WheelTraceParams[WheelIdx].SweepShape)
					{
					case ESweepShape::Spherecast:
					{
						float WheelRadius = PVehicle->Wheels[WheelIdx].GetEffectiveRadius();
						FVector VehicleUpAxis = TraceNormal;

						FVector Start = TraceStart + VehicleUpAxis * WheelRadius;
						FVector End = TraceEnd;

						if (OverlapResult.Component->SweepComponent(ComponentHit, Start, End, FQuat::Identity, FCollisionShape::MakeSphere(WheelRadius), TraceParams.bTraceComplex))
						{
							if (ComponentHit.Time < HitResult.Time)
							{
								HitResult = ComponentHit;
								HitResult.bBlockingHit = OverlapResult.bBlockingHit;
							}
						}
					}
					break;

					case ESweepShape::Raycast:
					default:
					{
						if (OverlapResult.Component->LineTraceComponent(ComponentHit, TraceStart, TraceEnd, TraceParams))
						{
							if (ComponentHit.Time < HitResult.Time)
							{
								HitResult = ComponentHit;
								HitResult.bBlockingHit = OverlapResult.bBlockingHit;
							}
						}
					}
					break;
					}
				}
			}
		}
	}
	else
	{
		SCOPE_CYCLE_COUNTER(STAT_ChaosVehicle_SuspensionTraces);
		for (int WheelIdx = 0; WheelIdx < SuspensionTrace.Num(); WheelIdx++)
		{
			FHitResult& HitResult = WheelState.TraceResult[WheelIdx];

			FVector TraceStart = SuspensionTrace[WheelIdx].Start;
			FVector TraceEnd = SuspensionTrace[WheelIdx].End;
			TraceParams.bTraceComplex = (WheelTraceParams[WheelIdx].SweepType == ESweepType::ComplexSweep);

			if (DynWheeledVehicleDebugParams.TraceTypeOverride > 0)
			{
				TraceParams.bTraceComplex = DynWheeledVehicleDebugParams.TraceTypeOverride == 2;
			}

			FVector TraceVector(TraceStart - TraceEnd); // reversed
			FVector TraceNormal = TraceVector.GetSafeNormal();

			switch (WheelTraceParams[WheelIdx].SweepShape)
			{
			case ESweepShape::Spherecast:
			{
				float WheelRadius = PVehicle->Wheels[WheelIdx].GetEffectiveRadius();
				FVector VehicleUpAxis = TraceNormal;

				World->SweepSingleByChannel(HitResult
					, TraceStart + VehicleUpAxis * WheelRadius
					, TraceEnd
					, FQuat::Identity, SpringCollisionChannel
					, FCollisionShape::MakeSphere(WheelRadius), TraceParams
					, ResponseParams);
			}
			break;



			case ESweepShape::Raycast:
			default:
			{
				World->LineTraceSingleByChannel(HitResult, TraceStart, TraceEnd, SpringCollisionChannel, TraceParams, ResponseParams);
			}
			break;
			}
		}
	}

}

void UDynamicVehicleSimulation::ApplyWheelFrictionForces(float DeltaTime)
{
	using namespace Chaos;

	for (int WheelIdx = 0; WheelIdx < PVehicle->Wheels.Num(); WheelIdx++)
	{
		auto& PWheel = PVehicle->Wheels[WheelIdx]; // Physics Wheel
		FHitResult& HitResult = WheelState.TraceResult[WheelIdx];

		if (PWheel.InContact())
		{
			if (HitResult.PhysMaterial.IsValid())
			{
				PWheel.SetSurfaceFriction(HitResult.PhysMaterial->Friction);
			}

			// take into account steering angle
			float SteerAngleDegrees = PWheel.SteeringAngle;
			FRotator SteeringRotator(0.f, SteerAngleDegrees, 0.f);
			FVector SteerLocalWheelVelocity = SteeringRotator.UnrotateVector(WheelState.LocalWheelVelocity[WheelIdx]);

			PWheel.SetVehicleGroundSpeed(SteerLocalWheelVelocity);
			PWheel.Simulate(DeltaTime);

			// Friction force calculations
			float RotationAngle = FMath::RadiansToDegrees(PWheel.GetAngularPosition());
			FVector FrictionForceLocal = PWheel.GetForceFromFriction();
			FrictionForceLocal = SteeringRotator.RotateVector(FrictionForceLocal);

			FVector GroundZVector = HitResult.Normal;
			FVector GroundXVector = FVector::CrossProduct(VehicleState.VehicleRightAxis, GroundZVector);
			FVector GroundYVector = FVector::CrossProduct(GroundZVector, GroundXVector);

			FMatrix Mat = FMatrix(GroundXVector, GroundYVector, GroundZVector, VehicleState.VehicleWorldTransform.GetLocation());
			FVector FrictionForceVector = Mat.TransformVector(FrictionForceLocal);

			check(PWheel.InContact());
			if (PVehicle->bLegacyWheelFrictionPosition)
			{
				AddForceAtPosition(FrictionForceVector, WheelState.WheelWorldLocation[WheelIdx]);
			}
			else
			{
				AddForceAtPosition(FrictionForceVector, HitResult.ImpactPoint);
			}

		}
		else
		{
			PWheel.SetVehicleGroundSpeed(FVector::ZeroVector);
			PWheel.SetWheelLoadForce(0.f);
			PWheel.Simulate(DeltaTime);
		}
	}
	PostProcessDifferentialLocking();

}

void UDynamicVehicleSimulation::PostProcessDifferentialLocking()
{
	using namespace Chaos;

	// Step 1: Calculate average Omega values based on the differential mode and system
	float frontOmega = 0.f;
	float rearOmega = 0.f;
	int frontWheelCount = 0;
	int rearWheelCount = 0;

	for (int WheelIdx = 0; WheelIdx < PVehicle->Wheels.Num(); WheelIdx++)
	{
		auto& PWheel = PVehicle->Wheels[WheelIdx];
		if (PWheel.Setup().AxleType == FSimpleWheelConfig::Front)
		{
			frontOmega += PWheel.GetAngularVelocity();
			frontWheelCount++;
		}
		else if (PWheel.Setup().AxleType == FSimpleWheelConfig::Rear)
		{
			rearOmega += PWheel.GetAngularVelocity();
			rearWheelCount++;
		}
	}

	float averageFrontOmega = frontWheelCount > 0 ? frontOmega / frontWheelCount : 0.f;
	float averageRearOmega = rearWheelCount > 0 ? rearOmega / rearWheelCount : 0.f;

	float SmoothingFactor = 0.1f; // Smoothing factor for Omega adjustments
	float SlipAllowance = 0.05f;  // Allow a small amount of slip for realism

	// Step 2: Apply post-processing based on the differential mode
	if (currentDifferentialMode == EDynamicDifferentialModes::AllDifferentials)
	{
		// Lock all wheels to the same Omega (average of all), with smoothing and slip allowance
		float overallAverageOmega = (frontWheelCount * averageFrontOmega + rearWheelCount * averageRearOmega) / (frontWheelCount + rearWheelCount);
		for (int WheelIdx = 0; WheelIdx < PVehicle->Wheels.Num(); WheelIdx++)
		{
			auto& PWheel = PVehicle->Wheels[WheelIdx];
			if (PWheel.EngineEnabled)
			{
				float smoothedOmega = FMath::Lerp(PWheel.GetAngularVelocity(), overallAverageOmega, SmoothingFactor);
				float slipAdjustedOmega = smoothedOmega * (1.0f + FMath::RandRange(-SlipAllowance, SlipAllowance));
				PWheel.SetAngularVelocity(slipAdjustedOmega);
			}
		}
	}
	else if (currentDifferentialMode == EDynamicDifferentialModes::RearDifferentialLock)
	{
		// Lock only the rear wheels to the same Omega (average of rear wheels), with smoothing and slip allowance
		for (int WheelIdx = 0; WheelIdx < PVehicle->Wheels.Num(); WheelIdx++)
		{
			auto& PWheel = PVehicle->Wheels[WheelIdx];
			if (PWheel.Setup().AxleType == FSimpleWheelConfig::Rear)
			{
				float smoothedOmega = FMath::Lerp(PWheel.GetAngularVelocity(), averageRearOmega, SmoothingFactor);
				float slipAdjustedOmega = smoothedOmega * (1.0f + FMath::RandRange(-SlipAllowance, SlipAllowance));
				PWheel.SetAngularVelocity(slipAdjustedOmega);
			}
		}
	}
	else if (currentDifferentialMode == EDynamicDifferentialModes::InterAxleLock)
	{
		// Ensure average of rear axle equals average of front axle, but keep individual wheels on each axle independent
		float interAxleOmega = (averageFrontOmega + averageRearOmega) / 2;

		for (int WheelIdx = 0; WheelIdx < PVehicle->Wheels.Num(); WheelIdx++)
		{
			auto& PWheel = PVehicle->Wheels[WheelIdx];
			if (PWheel.Setup().AxleType == FSimpleWheelConfig::Front)
			{
				float adjustmentFactor = averageFrontOmega != 0 ? interAxleOmega / averageFrontOmega : 1.0f;
				float smoothedOmega = FMath::Lerp(PWheel.GetAngularVelocity(), PWheel.GetAngularVelocity() * adjustmentFactor, SmoothingFactor);
				PWheel.SetAngularVelocity(smoothedOmega);
			}
			else if (PWheel.Setup().AxleType == FSimpleWheelConfig::Rear)
			{
				float adjustmentFactor = averageRearOmega != 0 ? interAxleOmega / averageRearOmega : 1.0f;
				float smoothedOmega = FMath::Lerp(PWheel.GetAngularVelocity(), PWheel.GetAngularVelocity() * adjustmentFactor, SmoothingFactor);
				PWheel.SetAngularVelocity(smoothedOmega);
			}
		}
	}
	else if (currentDifferentialMode == EDynamicDifferentialModes::OpenDifferential)
	{
		// No locking, wheels operate independently
	}
}



void UDynamicVehicleSimulation::ApplySuspensionForces(float DeltaTime, TArray<FWheelTraceParams>& WheelTraceParams)
{
	using namespace Chaos;

	TArray<float> SusForces;
	SusForces.Init(0.f, PVehicle->Suspension.Num());

	for (int WheelIdx = 0; WheelIdx < SusForces.Num(); WheelIdx++)
	{
		FHitResult& HitResult = WheelState.TraceResult[WheelIdx];

		float NewDesiredLength = 1.0f; // suspension max length
		float ForceMagnitude2 = 0.f;
		auto& PWheel = PVehicle->Wheels[WheelIdx];
		auto& PSuspension = PVehicle->Suspension[WheelIdx];
		float SuspensionMovePosition = -PSuspension.Setup().MaxLength;

		if (!DynWheeledVehicleDebugParams.DisableConstraintSuspension)
		{
			if (WheelIdx < ConstraintHandles.Num())
			{
				FPhysicsConstraintHandle& ConstraintHandle = ConstraintHandles[WheelIdx];
				if (ConstraintHandle.IsValid())
				{
					if (Chaos::FSuspensionConstraint* Constraint = static_cast<Chaos::FSuspensionConstraint*>(ConstraintHandle.Constraint))
					{
						if (FSuspensionConstraintPhysicsProxy* Proxy = Constraint->GetProxy<FSuspensionConstraintPhysicsProxy>())
						{
							FVec3 TargetPos;
							if (WheelTraceParams[WheelIdx].SweepShape == ESweepShape::Spherecast)
							{
								TargetPos = HitResult.Location;
							}
							else
							{
								TargetPos = HitResult.ImpactPoint + (PWheel.GetEffectiveRadius() * VehicleState.VehicleUpAxis);
							}

							Chaos::FPhysicsSolver* Solver = Proxy->GetSolver<Chaos::FPhysicsSolver>();

							Solver->SetSuspensionTarget(Constraint, TargetPos, HitResult.ImpactNormal, PWheel.InContact());
						}
					}
				}
			}
		}

		if (PWheel.InContact())
		{
			NewDesiredLength = HitResult.Distance;

			SuspensionMovePosition = -FVector::DotProduct(WheelState.WheelWorldLocation[WheelIdx] - HitResult.ImpactPoint, VehicleState.VehicleUpAxis) + PWheel.GetEffectiveRadius();

			PSuspension.SetSuspensionLength(NewDesiredLength, PWheel.GetEffectiveRadius());
			PSuspension.SetLocalVelocity(WheelState.LocalWheelVelocity[WheelIdx]);
			PSuspension.Simulate(DeltaTime);

			float ForceMagnitude = PSuspension.GetSuspensionForce();

			FVector GroundZVector = HitResult.Normal;
			FVector SuspensionForceVector = VehicleState.VehicleUpAxis * ForceMagnitude;

			FVector SusApplicationPoint = WheelState.WheelWorldLocation[WheelIdx] + PVehicle->Suspension[WheelIdx].Setup().SuspensionForceOffset;

			check(PWheel.InContact());
			if (DynWheeledVehicleDebugParams.DisableConstraintSuspension)
			{
				AddForceAtPosition(SuspensionForceVector, SusApplicationPoint);
			}

			ForceMagnitude = PSuspension.Setup().WheelLoadRatio * ForceMagnitude + (1.f - PSuspension.Setup().WheelLoadRatio) * PSuspension.Setup().RestingForce;
			PWheel.SetWheelLoadForce(ForceMagnitude);
			PWheel.SetMassPerWheel(RigidHandle->M() / PVehicle->Wheels.Num());
			SusForces[WheelIdx] = ForceMagnitude;

#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
			if (DynWheeledVehicleDebugParams.ShowSuspensionForces)
			{
				Chaos::FDebugDrawQueue::GetInstance().DrawDebugLine(
					SusApplicationPoint
					, SusApplicationPoint + SuspensionForceVector * DynVehicleDebugParams.ForceDebugScaling
					, FColor::Blue, false, -1.0f, 0, 5);

				Chaos::FDebugDrawQueue::GetInstance().DrawDebugLine(
					SusApplicationPoint
					, SusApplicationPoint + GroundZVector * 140.f
					, FColor::Yellow, false, -1.0f, 0, 5);
			}
#endif

		}
		else
		{
			PSuspension.SetSuspensionLength(PSuspension.GetTraceLength(PWheel.GetEffectiveRadius()), PWheel.Setup().WheelRadius);
			PWheel.SetWheelLoadForce(0.f);

		}

	}

	if (!DynWheeledVehicleDebugParams.DisableRollbarForces)
	{
		for (auto& Axle : PVehicle->GetAxles())
		{
			//#todo: only works with 2 wheels on an axle at present
			if (Axle.Setup.WheelIndex.Num() == 2)
			{
				uint16 WheelIdxA = Axle.Setup.WheelIndex[0];
				uint16 WheelIdxB = Axle.Setup.WheelIndex[1];

				float FV = Axle.Setup.RollbarScaling;
				float ForceDiffOnAxleF = SusForces[WheelIdxA] - SusForces[WheelIdxB];
				FVector ForceVector0 = VehicleState.VehicleUpAxis * ForceDiffOnAxleF * FV;
				FVector ForceVector1 = VehicleState.VehicleUpAxis * ForceDiffOnAxleF * -FV;

				FVector SusApplicationPoint0 = WheelState.WheelWorldLocation[WheelIdxA] + PVehicle->Suspension[WheelIdxA].Setup().SuspensionForceOffset;
				AddForceAtPosition(ForceVector0, SusApplicationPoint0);

				FVector SusApplicationPoint1 = WheelState.WheelWorldLocation[WheelIdxB] + PVehicle->Suspension[WheelIdxB].Setup().SuspensionForceOffset;
				AddForceAtPosition(ForceVector1, SusApplicationPoint1);
			}
		}
	}
}

void UDynamicVehicleSimulation::ProcessSteering(const FControlInputs& ControlInputs)
{
	using namespace Chaos;

	auto& PSteering = PVehicle->GetSteering();

	for (int WheelIdx = 0; WheelIdx < PVehicle->Wheels.Num(); WheelIdx++)
	{
		auto& PWheel = PVehicle->Wheels[WheelIdx]; // Physics Wheel
		FHitResult& HitResult = WheelState.TraceResult[WheelIdx];

		if (PWheel.SteeringEnabled)
		{
			float SpeedScale = 1.0f;

			// allow full counter steering when steering into a power slide
			//if (ControlInputs.SteeringInput * VehicleState.VehicleLocalVelocity.Y > 0.0f)
			{
				SpeedScale = PVehicle->GetSteering().GetSteeringFromVelocity(CmSToMPH(VehicleState.ForwardSpeed));
			}

			float SteeringAngle = ControlInputs.SteeringInput * SpeedScale;

			if (FMath::Abs(DynWheeledVehicleDebugParams.SteeringOverride) > 0.01f)
			{
				SteeringAngle = PWheel.MaxSteeringAngle * DynWheeledVehicleDebugParams.SteeringOverride;
			}
			else
			{
				float WheelSide = PVehicle->GetSuspension(WheelIdx).GetLocalRestingPosition().Y;
				SteeringAngle = PSteering.GetSteeringAngle(SteeringAngle, PWheel.MaxSteeringAngle, WheelSide);
			}

			PWheel.SetSteeringAngle(SteeringAngle);
		}
		else
		{
			PWheel.SetSteeringAngle(0.0f);
		}
	}
}

void UDynamicVehicleSimulation::ApplyInput(const FControlInputs& ControlInputs, float DeltaTime)
{
	using namespace Chaos;
	bool isIdleGearOROff = false;
	UChaosVehicleSimulation::ApplyInput(ControlInputs, DeltaTime);

	FControlInputs ModifiedInputs = ControlInputs;

	float EngineBraking = 0.f;
	if (PVehicle->HasTransmission() && PVehicle->HasEngine())
	{
		auto& PEngine = PVehicle->GetEngine();
		auto& PTransmission = PVehicle->GetTransmission();

		if (ModifiedInputs.TransmissionType != PTransmission.Setup().TransmissionType)
		{
			PTransmission.AccessSetup().TransmissionType = (Chaos::ETransmissionType)ModifiedInputs.TransmissionType;
		}

		if (ModifiedInputs.GearUpInput)
		{
			PTransmission.ChangeUp();
			ModifiedInputs.GearUpInput = false;
		}

		if (ModifiedInputs.GearDownInput)
		{
			PTransmission.ChangeDown();
			ModifiedInputs.GearDownInput = false;
		}

		if (DynWheeledVehicleDebugParams.ThrottleOverride > 0.f)
		{
			PTransmission.SetGear(1, true);
			ModifiedInputs.BrakeInput = 0.f;
			PEngine.SetThrottle(DynWheeledVehicleDebugParams.ThrottleOverride);
		}
		else
		{
				PEngine.SetThrottle(ModifiedInputs.ThrottleInput * ModifiedInputs.ThrottleInput);
		}

		if (PTransmission.GetCurrentGear() == 0)
			isIdleGearOROff = true;


		EngineBraking = PEngine.GetEngineRPM() * PEngine.Setup().EngineBrakeEffect;
	}

	for (int WheelIdx = 0; WheelIdx < PVehicle->Wheels.Num(); WheelIdx++)
	{
		auto& PWheel = PVehicle->Wheels[WheelIdx];

		float EngineBrakingForce = 0.0f;
		if (((ModifiedInputs.ThrottleInput < SMALL_NUMBER) && FMath::Abs(VehicleState.ForwardSpeed) > SMALL_NUMBER && PWheel.EngineEnabled) || dataImpactingSimulation.isBreakAssistActive || !dataImpactingSimulation.isEngineStarted)
		{
			EngineBrakingForce = EngineBraking;
		}

		if (PWheel.BrakeEnabled)
		{
			if (ModifiedInputs.ThrottleInput > SMALL_NUMBER && dataImpactingSimulation.isBreakAssistActive!=true)
			{
				EngineBraking = 0;
				EngineBrakingForce = 0.0f;
			}

			/*if(isIdleGearOROff)
			{
				EngineBraking = 0;
				EngineBrakingForce = 0.0f;
			}*/

			if (!dataImpactingSimulation.isEngineStarted && !isIdleGearOROff)
			{
				EngineBrakingForce = EngineBraking*2 + PWheel.HandbrakeTorque;
			}

			float BrakeForce = PWheel.MaxBrakeTorque * ModifiedInputs.BrakeInput;
			PWheel.SetBrakeTorque(TorqueMToCm(BrakeForce + EngineBrakingForce), FMath::Abs(EngineBrakingForce) > FMath::Abs(BrakeForce));
		}
		else if (ModifiedInputs.ThrottleInput > SMALL_NUMBER && dataImpactingSimulation.isBreakAssistActive != true)
		{
			EngineBraking = 0;
			EngineBrakingForce = 0.0f;
		}
		else /*if(!isIdleGearOROff)*/
		{
			PWheel.SetBrakeTorque(TorqueMToCm(EngineBraking), true);
		}

		if ((ModifiedInputs.HandbrakeInput && PWheel.HandbrakeEnabled) || ModifiedInputs.ParkingEnabled)
		{
			float HandbrakeForce = ModifiedInputs.ParkingEnabled ? PWheel.HandbrakeTorque : (ModifiedInputs.HandbrakeInput * PWheel.HandbrakeTorque);
			PWheel.SetBrakeTorque(TorqueMToCm(HandbrakeForce));
		}
		/*if (ModifiedInputs.BrakeInput == 0 && ModifiedInputs.HandbrakeInput == 0 && ModifiedInputs.ThrottleInput == 0)
		{
			PWheel.SetBrakeTorque(TorqueMToCm(0));
		}*/
	}


}

bool UDynamicVehicleSimulation::IsWheelSpinning() const
{
	if (!Chaos::FPhysicsSolverBase::IsNetworkPhysicsPredictionEnabled())
	{
		// Disable it for now since it is breaking determinism
		for (auto& Wheel : PVehicle->Wheels)
		{
			if (Wheel.IsSlipping())
			{
				return true;
			}
		}
	}

	return false;
}

//void UDynamicVehicleSimulation::ProcessMechanicalSimulation(float DeltaTime)
//{
//	using namespace Chaos;
//
//	if (PVehicle->HasEngine())
//	{
//		auto& PEngine = PVehicle->GetEngine();
//		auto& PTransmission = PVehicle->GetTransmission();
//		auto& PDifferential = PVehicle->GetDifferential();
//
//		float WheelRPM = 0;
//		for (int I = 0; I < PVehicle->Wheels.Num(); I++)
//		{
//			if (PVehicle->Wheels[I].EngineEnabled)
//			{
//				WheelRPM = FMath::Abs(PVehicle->Wheels[I].GetWheelRPM());
//			}
//		}
//
//		float WheelSpeedRPM = FMath::Abs(PTransmission.GetEngineRPMFromWheelRPM(WheelRPM));
//		float targetRPM = dataImpactingSimulation.engineData.EngineIdleRPM, maxAllowableRPM = dataImpactingSimulation.engineData.MaxRPM;
//		//Adjust RPM based on wheel RPM and fuel intake
//		if (dataImpactingSimulation.isFilled )
//		{
//			bool bypassMaxCheck = false;
//			float engineMaxRPM = dataImpactingSimulation.engineData.MaxRPM;
//
//			maxAllowableRPM = GetCalculatedRPM(WheelRPM);
//			targetRPM_BasedOnFuel = maxAllowableRPM;
//	
//			if (PTransmission.GetEngineRPMFromWheelRPM(WheelRPM) >= dataImpactingSimulation.engineData.EngineIdleRPM)
//			{
//				if (dataImpactingSimulation.isDownShifting || !dataImpactingSimulation.isThrottleActive)
//				{
//					bypassMaxCheck = true;
//				}
//			}
//			targetRPM = targetRPM_BasedOnFuel;
//			
//
//			if (maxAllowableRPM == dataImpactingSimulation.engineData.MaxRPM)
//				maxAllowableRPM = maxAllowableRPM - 1;
//			if (!bypassMaxCheck)
//			{
//				float maxRPMChangeRate = dataImpactingSimulation.engineData.RPM_IncreasRate;
//				float currentRPM = PEngine.GetEngineRPM();
//				float rpmDifference = targetRPM - currentRPM;
//
//
//				if (rpmDifference < 0)
//					maxRPMChangeRate = 10;
//				targetRPM = currentRPM + FMath::Sign(rpmDifference) * maxRPMChangeRate;
//			}
//
//			if (targetRPM > maxAllowableRPM && !bypassMaxCheck)
//			{
//				targetRPM = maxAllowableRPM;
//			}
//			if (targetRPM < dataImpactingSimulation.engineData.EngineIdleRPM)
//			{
//				targetRPM = dataImpactingSimulation.engineData.EngineIdleRPM;
//			}
//
//
//			PEngine.SetEngineRPM(false, targetRPM);
//		
//			
//		}
//		else
//		{
//			PEngine.SetEngineRPM(PTransmission.IsOutOfGear(), PTransmission.GetEngineRPMFromWheelRPM(WheelRPM));
//		}
//
//		PEngine.Simulate(DeltaTime);
//
//		PTransmission.SetEngineRPM(PEngine.GetEngineRPM()); // needs engine RPM to decide when to change gear (automatic gearbox)
//		PTransmission.SetAllowedToChangeGear(!VehicleState.bVehicleInAir && !IsWheelSpinning());
//		float GearRatio = PTransmission.GetGearRatio(PTransmission.GetCurrentGear());
//
//		PTransmission.Simulate(DeltaTime);
//		
//		float TransmissionTorque;
//		if (dataImpactingSimulation.isFilled && false)
//		{
//			if (PTransmission.GetCurrentGear() != 0 && PEngine.GetEngineTorque() == 0)
//			{
//				TransmissionTorque = PTransmission.GetTransmissionTorque(PEngine.GetTorqueFromRPM());
//			}
//			else
//			{
//				TransmissionTorque = PTransmission.GetTransmissionTorque(PEngine.GetEngineTorque());
//
//			}
//		}
//		else
//			TransmissionTorque = PTransmission.GetTransmissionTorque(PEngine.GetEngineTorque());
//
//
//		if (WheelSpeedRPM > PEngine.Setup().MaxRPM)
//		{
//			TransmissionTorque = 0.f;
//		}
//		// apply drive torque to wheels
//		for (int WheelIdx = 0; WheelIdx < PVehicle->Wheels.Num(); WheelIdx++)
//		{
//			auto& PWheel = PVehicle->Wheels[WheelIdx];
//			if (PWheel.Setup().EngineEnabled)
//			{
//				float modificationFactor = 1;
//				if (dataImpactingSimulation.isFilled)
//					modificationFactor = dataImpactingSimulation.driveTorqueIncreaseFactor;
//				PWheel.SetDriveTorque(TorqueMToCm(TransmissionTorque)* PWheel.Setup().TorqueRatio * modificationFactor);
//				
//			}
//			else
//			{
//				PWheel.SetDriveTorque(0.f);
//			}
//		}
//
//	}
//}

void UDynamicVehicleSimulation::ProcessMechanicalSimulation(float DeltaTime)
{
	using namespace Chaos;

	if (PVehicle->HasEngine())
	{
		auto& PEngine = PVehicle->GetEngine();
		auto& PTransmission = PVehicle->GetTransmission();
		auto& PDifferential = PVehicle->GetDifferential();

		float WheelRPM = 0;
		for (int I = 0; I < PVehicle->Wheels.Num(); I++)
		{
			if (PVehicle->Wheels[I].EngineEnabled)
			{
				WheelRPM = FMath::Abs(PVehicle->Wheels[I].GetWheelRPM());
			}
		}

		float WheelSpeedRPM = FMath::Abs(PTransmission.GetEngineRPMFromWheelRPM(WheelRPM));
		float targetRPM = dataImpactingSimulation.engineData.EngineIdleRPM, maxAllowableRPM = dataImpactingSimulation.engineData.MaxRPM;
		if (dataImpactingSimulation.isFilled)
		{
			bool bypassMaxCheck = false;
			float engineMaxRPM = dataImpactingSimulation.engineData.MaxRPM;

			maxAllowableRPM = GetCalculatedRPM(WheelRPM);
			targetRPM_BasedOnFuel = maxAllowableRPM;

			if (PTransmission.GetEngineRPMFromWheelRPM(WheelRPM) >= dataImpactingSimulation.engineData.EngineIdleRPM)
			{
				if (dataImpactingSimulation.isDownShifting || !dataImpactingSimulation.isThrottleActive)
				{
					bypassMaxCheck = true;
				}
			}
			targetRPM = targetRPM_BasedOnFuel;


			if (maxAllowableRPM == dataImpactingSimulation.engineData.MaxRPM)
				maxAllowableRPM = maxAllowableRPM - 1;
			if (!bypassMaxCheck)
			{
				float maxRPMChangeRate = dataImpactingSimulation.engineData.RPM_IncreasRate;
				float currentRPM = PEngine.GetEngineRPM();
				float rpmDifference = targetRPM - currentRPM;


				if (rpmDifference < 0)
					maxRPMChangeRate = 10;
				targetRPM = currentRPM + FMath::Sign(rpmDifference) * maxRPMChangeRate;
			}

			if (targetRPM > maxAllowableRPM && !bypassMaxCheck)
			{
				targetRPM = maxAllowableRPM;
			}
			if (targetRPM < dataImpactingSimulation.engineData.EngineIdleRPM)
			{
				targetRPM = dataImpactingSimulation.engineData.EngineIdleRPM;
			}


			PEngine.SetEngineRPM(false, targetRPM);


		}
		else
		{
			PEngine.SetEngineRPM(PTransmission.IsOutOfGear(), PTransmission.GetEngineRPMFromWheelRPM(WheelRPM));
		}

		PEngine.Simulate(DeltaTime);

		PTransmission.SetEngineRPM(PEngine.GetEngineRPM()); // needs engine RPM to decide when to change gear (automatic gearbox)
		PTransmission.SetAllowedToChangeGear(!VehicleState.bVehicleInAir && !IsWheelSpinning());
		float GearRatio = PTransmission.GetGearRatio(PTransmission.GetCurrentGear());

		PTransmission.Simulate(DeltaTime);

		float TransmissionTorque;
		if (dataImpactingSimulation.isFilled && false)
		{
			if (PTransmission.GetCurrentGear() != 0 && PEngine.GetEngineTorque() == 0)
			{
				TransmissionTorque = PTransmission.GetTransmissionTorque(PEngine.GetTorqueFromRPM());
			}
			else
			{
				TransmissionTorque = PTransmission.GetTransmissionTorque(PEngine.GetEngineTorque());
			}
		}
		else
			TransmissionTorque = PTransmission.GetTransmissionTorque(PEngine.GetEngineTorque());


		if (WheelSpeedRPM > PEngine.Setup().MaxRPM)
		{
			TransmissionTorque = 0.f;
		}

		float modificationFactor = 1;
		if (dataImpactingSimulation.isFilled)
		{
			modificationFactor = dataImpactingSimulation.driveTorqueIncreaseFactor;
		}


		if (currentDifferentialSystem == EDynamicVehicleDifferential::RearWheelDrive)
		{
			// Only apply torque to rear wheels
			for (int WheelIdx = 0; WheelIdx < PVehicle->Wheels.Num(); WheelIdx++)
			{
				auto& PWheel = PVehicle->Wheels[WheelIdx];
				if (PWheel.Setup().AxleType == FSimpleWheelConfig::Rear && PWheel.EngineEnabled)
				{
					PWheel.SetDriveTorque(TorqueMToCm(TransmissionTorque) * PWheel.Setup().TorqueRatio * modificationFactor);
				}
				else
				{
					PWheel.SetDriveTorque(0.f);
				}
			}
		}
		else if (currentDifferentialSystem == EDynamicVehicleDifferential::FrontWheelDrive)
		{
			// Only apply torque to front wheels
			for (int WheelIdx = 0; WheelIdx < PVehicle->Wheels.Num(); WheelIdx++)
			{
				auto& PWheel = PVehicle->Wheels[WheelIdx];
				if (PWheel.Setup().AxleType == FSimpleWheelConfig::Front && PWheel.EngineEnabled)
				{
					PWheel.SetDriveTorque(TorqueMToCm(TransmissionTorque) * PWheel.Setup().TorqueRatio * modificationFactor);
				}
				else
				{
					PWheel.SetDriveTorque(0.f);
				}
			}
		}
		else if (currentDifferentialSystem == EDynamicVehicleDifferential::AllWheelDrive)
		{
			float frontSlip = 0.f;
			float rearSlip = 0.f;
			int frontWheelCount = 0;
			int rearWheelCount = 0;

			for (int WheelIdx = 0; WheelIdx < PVehicle->Wheels.Num(); WheelIdx++)
			{
				auto& PWheel = PVehicle->Wheels[WheelIdx];
				if (PWheel.Setup().AxleType == FSimpleWheelConfig::Front)
				{
					frontSlip += PWheel.GetSlipMagnitude();
					frontWheelCount++;
				}
				else if (PWheel.Setup().AxleType == FSimpleWheelConfig::Rear)
				{
					rearSlip += PWheel.GetSlipMagnitude();
					rearWheelCount++;
				}
			}

			float totalSlip = frontSlip + rearSlip;
			float frontBias = 0.5f;
			float rearBias = 0.5f;

			if (totalSlip > 1.0f)
			{
				frontBias = rearSlip / totalSlip; 
				rearBias = frontSlip / totalSlip;  
			}

			float biasSum = (frontBias * frontWheelCount + rearBias * rearWheelCount) / (frontWheelCount + rearWheelCount);
			frontBias /= biasSum;
			rearBias /= biasSum;

			if (!bTorqueLimiterInitialized)
			{
				TimeSinceStart = 0.0f;
				TorqueRampUpTime = 2.0f;
				bTorqueLimiterInitialized = true;
			}
			TimeSinceStart += DeltaTime;

			float rampUpFactor = FMath::Clamp(TimeSinceStart / TorqueRampUpTime, 0.0f, 1.0f);
			float limitedTransmissionTorque = rampUpFactor * TransmissionTorque;

			for (int WheelIdx = 0; WheelIdx < PVehicle->Wheels.Num(); WheelIdx++)
			{
				auto& PWheel = PVehicle->Wheels[WheelIdx];
				if (PWheel.EngineEnabled)
				{
					float accelerationMagnitude = FMath::Abs(VehicleState.ForwardSpeed);
					if (accelerationMagnitude > AccelerationThreshold)
					{
						frontBias = FMath::Min(1.0f, frontBias + BiasAdjustmentRate * DeltaTime);
						rearBias = FMath::Max(0.0f, rearBias - BiasAdjustmentRate * DeltaTime);
					}

					float adjustedModificationFactor = (PWheel.Setup().AxleType == FSimpleWheelConfig::Front) ? frontBias : rearBias;
					float combinedModificationFactor = modificationFactor * adjustedModificationFactor;
					PWheel.SetDriveTorque(TorqueMToCm(limitedTransmissionTorque) * PWheel.Setup().TorqueRatio * combinedModificationFactor);
				}
				else
				{
					PWheel.SetDriveTorque(0.f);
				}
			}
		}
		else if (currentDifferentialSystem == EDynamicVehicleDifferential::Undefined)
		{
			for (int WheelIdx = 0; WheelIdx < PVehicle->Wheels.Num(); WheelIdx++)
			{
				PVehicle->Wheels[WheelIdx].SetDriveTorque(0.f);
			}
		}
	}
}

void UDynamicVehicleSimulation::DrawDebug3D()
{
	using namespace Chaos;

#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)

	UChaosVehicleSimulation::DrawDebug3D();

	if (PVehicle == nullptr)
	{
		return;
	}

	const FTransform BodyTransform = VehicleState.VehicleWorldTransform;

	if (DynWheeledVehicleDebugParams.ShowSuspensionLimits)
	{
		for (int WheelIdx = 0; WheelIdx < PVehicle->Suspension.Num(); WheelIdx++)
		{
			auto& PSuspension = PVehicle->Suspension[WheelIdx];
			auto& PWheel = PVehicle->Wheels[WheelIdx];
			// push the visualization out a bit sideways from the wheel model so we can actually see it
			FVector VehicleRightAxis = VehicleState.VehicleWorldTransform.GetUnitAxis(EAxis::Y) * 48.0f;
			const FVector& WheelOffset = PSuspension.GetLocalRestingPosition();
			if (WheelOffset.Y < 0.0f)
			{
				VehicleRightAxis = VehicleRightAxis * -1.0f;
			}

			FVector LocalDirection = PSuspension.Setup().SuspensionAxis;
			FVector WorldLocation = BodyTransform.TransformPosition(WheelOffset);
			FVector WorldDirection = BodyTransform.TransformVector(LocalDirection);

			FVector Start = WorldLocation + WorldDirection * (PWheel.GetEffectiveRadius() - PSuspension.Setup().SuspensionMaxRaise);
			FVector End = WorldLocation + WorldDirection * (PWheel.GetEffectiveRadius() + PSuspension.Setup().SuspensionMaxDrop);

			Chaos::FDebugDrawQueue::GetInstance().DrawDebugLine(
				Start + VehicleRightAxis, End + VehicleRightAxis, FColor::Orange, false, -1.f, 0, 3.f);

			FVector Start2 = WorldLocation - WorldDirection * PSuspension.Setup().SuspensionMaxRaise;
			FVector End2 = WorldLocation + WorldDirection * PSuspension.Setup().SuspensionMaxDrop;

			FDebugDrawQueue::GetInstance().DrawDebugLine(Start2 + VehicleRightAxis, End2 + VehicleRightAxis, FColor::Yellow, false, -1.f, 0, 3.f);
		}
	}

	if (DynWheeledVehicleDebugParams.ShowRaycastComponent)
	{
		for (int WheelIdx = 0; WheelIdx < PVehicle->Suspension.Num(); WheelIdx++)
		{
			FHitResult& Hit = WheelState.TraceResult[WheelIdx];

			FVector VehicleRightAxis = VehicleState.VehicleWorldTransform.GetUnitAxis(EAxis::Y) * 20.0f;
			FVector VehicleUpAxis = VehicleState.VehicleWorldTransform.GetUnitAxis(EAxis::Z) * 20.0f;
			const FVector& WheelOffset = PVehicle->Suspension[WheelIdx].GetLocalRestingPosition();
			if (WheelOffset.Y < 0.0f)
			{
				VehicleRightAxis = VehicleRightAxis * -1.0f;
			}

			FVector Pt = Hit.ImpactPoint + VehicleRightAxis;
			if (Hit.GetComponent())
			{
				FDebugDrawQueue::GetInstance().DrawDebugString(Pt + VehicleRightAxis, Hit.GetComponent()->GetName(), nullptr, FColor::White, -1.f, true, 1.0f);
			}
		}
	}

	if (DynWheeledVehicleDebugParams.ShowRaycastMaterial)
	{
		for (int WheelIdx = 0; WheelIdx < PVehicle->Suspension.Num(); WheelIdx++)
		{
			FHitResult& Hit = WheelState.TraceResult[WheelIdx];

			FVector VehicleRightAxis = VehicleState.VehicleWorldTransform.GetUnitAxis(EAxis::Y) * 20.0f;
			FVector VehicleUpAxis = VehicleState.VehicleWorldTransform.GetUnitAxis(EAxis::Z) * 20.0f;
			const FVector& WheelOffset = PVehicle->Suspension[WheelIdx].GetLocalRestingPosition();
			if (WheelOffset.Y < 0.0f)
			{
				VehicleRightAxis = VehicleRightAxis * -1.0f;
			}

			FVector Pt = Hit.ImpactPoint + VehicleRightAxis;
			if (Hit.PhysMaterial.IsValid())
			{
				FDebugDrawQueue::GetInstance().DrawDebugString(Pt + VehicleRightAxis + VehicleUpAxis, Hit.PhysMaterial->GetName(), nullptr, FColor::White, -1.f, true, 1.0f);
			}
		}

	}

	if (DynWheeledVehicleDebugParams.ShowWheelCollisionNormal)
	{
		FString Name;
		for (int WheelIdx = 0; WheelIdx < PVehicle->Suspension.Num(); WheelIdx++)
		{
			FHitResult& Hit = WheelState.TraceResult[WheelIdx];

			FVector VehicleRightAxis = VehicleState.VehicleWorldTransform.GetUnitAxis(EAxis::Y) * 20.0f;
			const FVector& WheelOffset = PVehicle->Suspension[WheelIdx].GetLocalRestingPosition();
			if (WheelOffset.Y < 0.0f)
			{
				VehicleRightAxis = VehicleRightAxis * -1.0f;
			}

			FVector Pt = Hit.ImpactPoint + VehicleRightAxis;
			FDebugDrawQueue::GetInstance().DrawDebugLine(Pt, Pt + Hit.Normal * 20.0f, FColor::Yellow, false, 1.0f, 0, 1.0f);
			FDebugDrawQueue::GetInstance().DrawDebugSphere(Pt, 5.0f, 4, FColor::White, false, 1.0f, 0, 1.0f);
		}
	}

	if (DynWheeledVehicleDebugParams.ShowSuspensionRaycasts)
	{
		for (int WheelIdx = 0; WheelIdx < PVehicle->Suspension.Num(); WheelIdx++)
		{
			const FVector& TraceStart = WheelState.Trace[WheelIdx].Start;
			const FVector& TraceEnd = WheelState.Trace[WheelIdx].End;

			// push the visualization out a bit sideways from the wheel model so we can actually see it
			FVector VehicleRightAxis = VehicleState.VehicleWorldTransform.GetUnitAxis(EAxis::Y) * 50.0f;
			const FVector& WheelOffset = PVehicle->Suspension[WheelIdx].GetLocalRestingPosition();
			if (WheelOffset.Y < 0.0f)
			{
				VehicleRightAxis = VehicleRightAxis * -1.0f;
			}

			FColor UseColor = PVehicle->Wheels[WheelIdx].InContact() ? FColor::Green : FColor::Red;
			Chaos::FDebugDrawQueue::GetInstance().DrawDebugDirectionalArrow(TraceStart + VehicleRightAxis, TraceEnd + VehicleRightAxis, 10.f, UseColor, false, -1.f, 0, 2.f);

			Chaos::FDebugDrawQueue::GetInstance().DrawDebugLine(TraceStart, TraceStart + VehicleRightAxis, FColor::White, false, -1.f, 0, 1.f);
			Chaos::FDebugDrawQueue::GetInstance().DrawDebugLine(TraceEnd, TraceEnd + VehicleRightAxis, FColor::White, false, -1.f, 0, 1.f);
		}
	}
#endif
}

float UDynamicVehicleSimulation::GetRelativeEngineRPM_FromSpeed(float currentSpeed)
{
	if (PVehicle->HasEngine())
	{
		auto& PEngine = PVehicle->GetEngine();
		auto& PTransmission = PVehicle->GetTransmission();
		
		float engineRPM = 0;
		currentSpeed /= 2; 
		if (PTransmission.GetCurrentGear() > 0)
			engineRPM = UKismetMathLibrary::MapRangeClamped(currentSpeed, dataImpactingSimulation.transmissionData.GetMinimumSpeedForGear((PTransmission.GetCurrentGear() + 1) / 2 - 1, true),
				dataImpactingSimulation.transmissionData.GetMaximumSpeedForGear((PTransmission.GetCurrentGear() + 1) / 2 - 1, true),
				0, dataImpactingSimulation.engineData.MaxRPM);
		else if(PTransmission.GetCurrentGear() < 0)
			engineRPM = UKismetMathLibrary::MapRangeClamped(currentSpeed, dataImpactingSimulation.transmissionData.GetMinimumSpeedForGear((PTransmission.GetCurrentGear() - 1) / 2 + 1, false),
				dataImpactingSimulation.transmissionData.GetMaximumSpeedForGear((PTransmission.GetCurrentGear() - 1) / 2 + 1, false),
				0, dataImpactingSimulation.engineData.MaxRPM);

		return engineRPM;
	}
	return 0;

}

void UDynamicVehicleSimulation::SetDifferentialMode(EDynamicDifferentialModes newDifferentialMode)
{
	currentDifferentialMode = newDifferentialMode;
}

void UDynamicVehicleSimulation::SetDifferentialSystem(EDynamicVehicleDifferential newDifferentialSystem)
{
	currentDifferentialSystem = newDifferentialSystem;
}

float UDynamicVehicleSimulation::GetCalculatedRPM(float WheelRPM)
{
	float finalRPM = 0;
	auto& PTransmission = PVehicle->GetTransmission();
	if (dataImpactingSimulation.isFilled)
	{
		if (dataImpactingSimulation.engineData.RPM_DependsOnFuelInput)
		{
			finalRPM = UKismetMathLibrary::MapRangeClamped(dataImpactingSimulation.netFuelIntakeValue, dataImpactingSimulation.netFuelIntakeIdleRPMRequirement, dataImpactingSimulation.netFuelIntakeMaxRange,
															dataImpactingSimulation.engineData.EngineIdleRPM, dataImpactingSimulation.engineData.MaxRPM);

			
			float RPM2 =FMath::Abs( PTransmission.GetEngineRPMFromWheelRPM(WheelRPM));

			finalRPM = FMath::Min(finalRPM, RPM2);
		}
		else
		{
			finalRPM = PTransmission.GetEngineRPMFromWheelRPM(WheelRPM);
		}

		int transmissionGear = PTransmission.GetCurrentGear();
		int convertedGear = dataImpactingSimulation.GetActiveGear(transmissionGear, true);
		if (transmissionGear > 0 && convertedGear >= dataImpactingSimulation.maxGearLock)
		{
			finalRPM = FMath::Min(finalRPM, dataImpactingSimulation.transmissionData.ChangeUpRPM - 2);
		}
	}
	else
	{
		finalRPM = PTransmission.GetEngineRPMFromWheelRPM(WheelRPM);
	}

	return FMath::Clamp(finalRPM, dataImpactingSimulation.engineData.EngineIdleRPM, dataImpactingSimulation.engineData.MaxRPM);
	
}

void UDynamicVehicleSimulation::FillOutputState(FChaosVehicleAsyncOutput& Output)
{
	// #Note: remember to copy/interpolate values from the physics thread output in UChaosVehicleMovementComponent::ParallelUpdate
	const auto& VehicleWheels = PVehicle->Wheels;
	auto& VehicleSuspension = PVehicle->Suspension;
	if (PVehicle->HasTransmission())
	{
		auto& Transmission = PVehicle->GetTransmission();
		Output.VehicleSimOutput.CurrentGear = Transmission.GetCurrentGear();
		Output.VehicleSimOutput.TargetGear = Transmission.GetTargetGear();
		Output.VehicleSimOutput.TransmissionRPM = Transmission.GetTransmissionRPM();
		Output.VehicleSimOutput.TransmissionTorque = Transmission.GetTransmissionTorque(PVehicle->GetEngine().GetTorqueFromRPM(false));
	}
	if (PVehicle->HasEngine())
	{
		auto& Engine = PVehicle->GetEngine();
		Output.VehicleSimOutput.EngineRPM = Engine.GetEngineRPM();
		Output.VehicleSimOutput.EngineTorque = Engine.GetEngineTorque();
	}

	// #TODO: can we avoid copies when async is turned off
	for (int WheelIdx = 0; WheelIdx < VehicleWheels.Num(); WheelIdx++)
	{
		FWheelsOutput WheelsOut;
		WheelsOut.InContact = VehicleWheels[WheelIdx].InContact();
		WheelsOut.SteeringAngle = VehicleWheels[WheelIdx].GetSteeringAngle();
		WheelsOut.AngularPosition = VehicleWheels[WheelIdx].GetAngularPosition();
		WheelsOut.AngularVelocity = VehicleWheels[WheelIdx].GetAngularVelocity();
		WheelsOut.WheelRadius = VehicleWheels[WheelIdx].GetEffectiveRadius();

		WheelsOut.LateralAdhesiveLimit = VehicleWheels[WheelIdx].LateralAdhesiveLimit;
		WheelsOut.LongitudinalAdhesiveLimit = VehicleWheels[WheelIdx].LongitudinalAdhesiveLimit;

		WheelsOut.bIsSlipping = VehicleWheels[WheelIdx].IsSlipping();
		WheelsOut.SlipMagnitude = VehicleWheels[WheelIdx].GetSlipMagnitude();
		WheelsOut.bIsSkidding = VehicleWheels[WheelIdx].IsSkidding();
		WheelsOut.SkidMagnitude = VehicleWheels[WheelIdx].GetSkidMagnitude();
		WheelsOut.SkidNormal = WheelState.WorldWheelVelocity[WheelIdx].GetSafeNormal();
		WheelsOut.SlipAngle = VehicleWheels[WheelIdx].GetSlipAngle();

		WheelsOut.SuspensionOffset = VehicleSuspension[WheelIdx].GetSuspensionOffset();
		WheelsOut.SpringForce = VehicleSuspension[WheelIdx].GetSuspensionForce();
		WheelsOut.NormalizedSuspensionLength = VehicleSuspension[WheelIdx].GetNormalizedLength();

		WheelsOut.DriveTorque = Chaos::TorqueCmToM(VehicleWheels[WheelIdx].GetDriveTorque());
		WheelsOut.BrakeTorque = Chaos::TorqueCmToM(VehicleWheels[WheelIdx].GetBrakeTorque());

		WheelsOut.bABSActivated = VehicleWheels[WheelIdx].IsABSActivated();
		WheelsOut.bBlockingHit = WheelState.TraceResult[WheelIdx].bBlockingHit;
		WheelsOut.ImpactPoint = WheelState.TraceResult[WheelIdx].ImpactPoint;
		WheelsOut.HitLocation = WheelState.TraceResult[WheelIdx].Location;
		WheelsOut.PhysMaterial = WheelState.TraceResult[WheelIdx].PhysMaterial;

		Output.VehicleSimOutput.Wheels.Add(WheelsOut);
	}

}

void UDynamicVehicleSimulation::UpdateConstraintHandles(TArray<FPhysicsConstraintHandle>& ConstraintHandlesIn)
{
	UChaosVehicleSimulation::UpdateConstraintHandles(ConstraintHandlesIn);
	ConstraintHandles = ConstraintHandlesIn;
}

void UDynamicVehicleMovementComponent::Serialize(FArchive& Ar)
{
	Super::Serialize(Ar);

	Ar.UsingCustomVersion(FUE5MainStreamObjectVersion::GUID);
}

void UDynamicVehicleMovementComponent::PostLoad()
{
	Super::PostLoad();

#if WITH_EDITORONLY_DATA

	if (GetLinkerCustomVersion(FUE5MainStreamObjectVersion::GUID) < FUE5MainStreamObjectVersion::VehicleFrictionForcePositionChange)
	{
		bLegacyWheelFrictionPosition = true;
	}

#endif  // #if WITH_EDITORONLY_DATA

}


void UDynamicVehicleMovementComponent::FixupSkeletalMesh()
{
	Super::FixupSkeletalMesh();

	if (USkeletalMeshComponent* Mesh = Cast<USkeletalMeshComponent>(GetMesh()))
	{
		if (UPhysicsAsset* PhysicsAsset = Mesh->GetPhysicsAsset())
		{
			for (int32 WheelIdx = 0; WheelIdx < (useSystem1ForDifferential?(WheelSetupsForDifferentialSystem1):(WheelSetupsForDifferentialSystem2)).Num(); ++WheelIdx)
			{
				FDynamicWheelSetup& WheelSetup = (useSystem1ForDifferential?(WheelSetupsForDifferentialSystem1):(WheelSetupsForDifferentialSystem2))[WheelIdx];
				if (WheelSetup.BoneName != NAME_None)
				{
					int32 BodySetupIdx = PhysicsAsset->FindBodyIndex(WheelSetup.BoneName);

					if (BodySetupIdx >= 0 && (BodySetupIdx < Mesh->Bodies.Num()))
					{
						FBodyInstance* BodyInstanceWheel = Mesh->Bodies[BodySetupIdx];
						BodyInstanceWheel->SetResponseToAllChannels(ECR_Ignore);	//turn off collision for wheel automatically

						if (UBodySetup* BodySetup = PhysicsAsset->SkeletalBodySetups[BodySetupIdx])
						{

							{
								BodyInstanceWheel->SetInstanceSimulatePhysics(false);
								//BodyInstanceWheel->SetCollisionEnabled(ECollisionEnabled::NoCollision);
							}

							bool DeleteOriginalWheelConstraints = true;
							if (DeleteOriginalWheelConstraints)
							{
								//and get rid of constraints on the wheels. TODO: right now we remove all wheel constraints, we probably only want to remove parent constraints
								TArray<int32> WheelConstraints;
								PhysicsAsset->BodyFindConstraints(BodySetupIdx, WheelConstraints);
								for (int32 ConstraintIdx = 0; ConstraintIdx < WheelConstraints.Num(); ++ConstraintIdx)
								{
									FConstraintInstance* ConInst = Mesh->Constraints[WheelConstraints[ConstraintIdx]];
									ConInst->TermConstraint();
								}
							}
						}

					}

					if (!DynWheeledVehicleDebugParams.DisableConstraintSuspension)
					{
						FBodyInstance* TargetInstance = UpdatedPrimitive->GetBodyInstance();
						if (TargetInstance)
						{
							FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis)
								{
									const FVector LocalWheel = GetWheelRestingPosition(WheelSetup);
									FPhysicsConstraintHandle ConstraintHandle = FPhysicsInterface::CreateSuspension(Chassis, LocalWheel);

									if (ConstraintHandle.IsValid())
									{
										UDynamicWheel* Wheel = Wheels[WheelIdx];
										check(Wheel);
										ConstraintHandles.Add(ConstraintHandle);
										if (Chaos::FSuspensionConstraint* Constraint = static_cast<Chaos::FSuspensionConstraint*>(ConstraintHandle.Constraint))
										{
											Constraint->SetHardstopStiffness(1.0f);
											Constraint->SetSpringStiffness(Chaos::MToCm(Wheel->SpringRate) * 0.25f);
											Constraint->SetSpringPreload(Chaos::MToCm(Wheel->SpringPreload));
											Constraint->SetSpringDamping(Wheel->SuspensionDampingRatio * 5.0f);
											Constraint->SetMinLength(-Wheel->SuspensionMaxRaise);
											Constraint->SetMaxLength(Wheel->SuspensionMaxDrop);
											Constraint->SetAxis(-Wheel->SuspensionAxis);
										}
									}
								});
						}
					}

				}
			}
		}
		VehicleSimulationPT->UpdateConstraintHandles(ConstraintHandles); // TODO: think of a better way to communicate this data

		Mesh->KinematicBonesUpdateType = EKinematicBonesUpdateToPhysics::SkipSimulatingBones;

	}

}


bool UDynamicVehicleMovementComponent::CanCreateVehicle() const
{
	if (!Super::CanCreateVehicle())
		return false;

	check(GetOwner());
	FString ActorName = GetOwner()->GetName();

	for (int32 WheelIdx = 0; WheelIdx < (useSystem1ForDifferential?(WheelSetupsForDifferentialSystem1):(WheelSetupsForDifferentialSystem2)).Num(); ++WheelIdx)
	{
		const FDynamicWheelSetup& WheelSetup = (useSystem1ForDifferential?(WheelSetupsForDifferentialSystem1):(WheelSetupsForDifferentialSystem2))[WheelIdx];

		if (WheelSetup.WheelClass == NULL)
		{
			UE_LOG(LogTemp, Warning, TEXT("Can't create vehicle %s (%s). Wheel %d is not set."), *ActorName, *GetPathName(), WheelIdx);
			return false;
		}

		if (WheelSetup.BoneName == NAME_None)
		{
			UE_LOG(LogTemp, Warning, TEXT("Can't create vehicle %s (%s). Bone name for wheel %d is not set."), *ActorName, *GetPathName(), WheelIdx);
			return false;
		}

	}

	return true;
}


void UDynamicVehicleMovementComponent::OnCreatePhysicsState()
{
	Super::OnCreatePhysicsState();
}

void UDynamicVehicleMovementComponent::CreateVehicle()
{
	Super::CreateVehicle();

	if (PVehicleOutput)
	{
		CreateWheels();

		// Need to bind to the notify delegate on the mesh in case physics state is changed
		if (USkeletalMeshComponent* MeshComp = GetSkeletalMesh())
		{
			MeshOnPhysicsStateChangeHandle = MeshComp->RegisterOnPhysicsCreatedDelegate(FOnSkelMeshPhysicsCreated::CreateUObject(this, &UDynamicVehicleMovementComponent::RecreatePhysicsState));
			if (UDynamicVehicleAnimationInstance* VehicleAnimInstance = Cast<UDynamicVehicleAnimationInstance>(MeshComp->GetAnimInstance()))
			{
				VehicleAnimInstance->SetWheeledVehicleComponent(this);
			}
		}
	}

	if (VehicleSimulationPT.Get())
	{
		derivedPtrForSimulationClass = static_cast<UDynamicVehicleSimulation*>(VehicleSimulationPT.Get());
		if (derivedPtrForSimulationClass)
		{
			UE_LOG(LogTemp, Display, TEXT("Derived Ptr has been assigned a copy of the Dynamic Simulation Class for Data transfer"));
			derivedPtrForSimulationClass->dataImpactingSimulation = FDynamicSimulationData(0, gasPedalMinValue, gasPedalMaxValue, fuelValueToSustainIdleRPM, TransmissionSetup, EngineSetup ,vehicleFunctionalities, true);
		}
	}

	TransmissionSetup.SetTransferCaseModifier(transferCaseConfig.GetTransferCaseRatio());

}

void UDynamicVehicleMovementComponent::OnDestroyPhysicsState()
{
	if (PVehicleOutput.IsValid())
	{
		if (MeshOnPhysicsStateChangeHandle.IsValid())
		{
			if (USkeletalMeshComponent* MeshComp = GetSkeletalMesh())
			{
				MeshComp->UnregisterOnPhysicsCreatedDelegate(MeshOnPhysicsStateChangeHandle);
			}
		}

		DestroyWheels();

		if (ConstraintHandles.Num() > 0)
		{
			for (FPhysicsConstraintHandle ConstraintHandle : ConstraintHandles)
			{
				FPhysicsCommand::ExecuteWrite(ConstraintHandle, [&](const FPhysicsConstraintHandle& Constraint)
					{
						FPhysicsInterface::ReleaseConstraint(ConstraintHandle);
					});
			}
		}
		ConstraintHandles.Empty();
	}

	Super::OnDestroyPhysicsState();

}

void UDynamicVehicleMovementComponent::NextDebugPage()
{
	/*int PageAsInt = (int)DebugPage;
	PageAsInt++;
	if (PageAsInt >= EDynamicDebugPages::MaxDebugPages)
	{
		PageAsInt = 0;
	}
	DebugPage = (EDynamicDebugPages)PageAsInt;*/
}

void UDynamicVehicleMovementComponent::PrevDebugPage()
{
	/*int PageAsInt = (int)DebugPage;
	PageAsInt--;
	if (PageAsInt < 0)
	{
		PageAsInt = EDynamicDebugPages::MaxDebugPages - 1;
	}
	DebugPage = (EDynamicDebugPages)PageAsInt;*/
}

void UDynamicVehicleMovementComponent::ComputeConstants()
{
	Super::ComputeConstants();
}

void UDynamicVehicleMovementComponent::CreateWheels()
{
	// Wheels num is getting copied when blueprint recompiles, so we have to manually reset here
	Wheels.Reset();

	// Instantiate the wheels
	for (int32 WheelIdx = 0; WheelIdx < (useSystem1ForDifferential?(WheelSetupsForDifferentialSystem1):(WheelSetupsForDifferentialSystem2)).Num(); ++WheelIdx)
	{
		UDynamicWheel* Wheel = NewObject<UDynamicWheel>(this, (useSystem1ForDifferential?(WheelSetupsForDifferentialSystem1):(WheelSetupsForDifferentialSystem2))[WheelIdx].WheelClass);
		check(Wheel);

		Wheels.Add(Wheel);
	}

	// Initialize the wheels
	for (int32 WheelIdx = 0; WheelIdx < Wheels.Num(); ++WheelIdx)
	{
		Wheels[WheelIdx]->Init(this, WheelIdx);
	}

	WheelStatus.SetNum((useSystem1ForDifferential?(WheelSetupsForDifferentialSystem1):(WheelSetupsForDifferentialSystem2)).Num());
	CachedState.SetNum((useSystem1ForDifferential?(WheelSetupsForDifferentialSystem1):(WheelSetupsForDifferentialSystem2)).Num());

	RecalculateAxles();
}

void UDynamicVehicleMovementComponent::DestroyWheels()
{
	for (int32 i = 0; i < Wheels.Num(); ++i)
	{
		Wheels[i]->Shutdown();
	}

	Wheels.Reset();
}

void UDynamicVehicleMovementComponent::SetupVehicle(TUniquePtr<Chaos::FSimpleWheeledVehicle>& PVehicle)
{
	using namespace Chaos;

	check(PVehicle);

	Super::SetupVehicle(PVehicle);
	NumDrivenWheels = 0;

	// we are allowed any number of wheels not limited to only 4
	for (int32 WheelIdx = 0; WheelIdx < (useSystem1ForDifferential?(WheelSetupsForDifferentialSystem1):(WheelSetupsForDifferentialSystem2)).Num(); ++WheelIdx)
	{
		UDynamicWheel* Wheel = (useSystem1ForDifferential?(WheelSetupsForDifferentialSystem1):(WheelSetupsForDifferentialSystem2))[WheelIdx].WheelClass.GetDefaultObject();

		//if (SteeringSetup.useOverrideSteerAngle)
		//{
		//	if (Wheel->bAffectedBySteering)
		//	{
		//		Wheel->MaxSteerAngle = SteeringSetup.overrideSteerAngle;
		//	}
		//}		

		// create Dynamic states passing in pointer to their Static setup data
		Chaos::FSimpleWheelSim WheelSim(&Wheel->GetPhysicsWheelConfig());

		if (Wheel->GetAxleType() != EDynamicAxleType::Undefined)
		{
			bool EngineEnable = false;
			if (Wheel->GetAxleType() == EDynamicAxleType::Front)
			{
				if (DifferentialSetup.GetDifferentialTypefromActiveSystem(useSystem1ForDifferential) == EDynamicVehicleDifferential::AllWheelDrive
					|| DifferentialSetup.GetDifferentialTypefromActiveSystem(useSystem1ForDifferential) == EDynamicVehicleDifferential::FrontWheelDrive)
				{
					EngineEnable = true;
				}
			}
			else if (Wheel->GetAxleType() == EDynamicAxleType::Rear)
			{
				if (DifferentialSetup.GetDifferentialTypefromActiveSystem(useSystem1ForDifferential) == EDynamicVehicleDifferential::AllWheelDrive
					|| DifferentialSetup.GetDifferentialTypefromActiveSystem(useSystem1ForDifferential) == EDynamicVehicleDifferential::RearWheelDrive)
				{
					EngineEnable = true;
				}
			}

			WheelSim.AccessSetup().EngineEnabled = EngineEnable;
		}

		WheelSim.SetWheelRadius(Wheel->WheelRadius); // initial radius
		PVehicle->Wheels.Add(WheelSim);

		FWheelsOutput WheelsOutput; // Receptacle for Data coming out of physics simulation on physics thread
		PVehicleOutput->Wheels.Add(WheelsOutput);

		Chaos::FSimpleSuspensionSim SuspensionSim(&Wheel->GetPhysicsSuspensionConfig());
		PVehicle->Suspension.Add(SuspensionSim);

		if (WheelSim.Setup().EngineEnabled)
		{
			NumDrivenWheels++;
		}

		// for debugging to identify a single wheel
		PVehicle->Wheels[WheelIdx].SetWheelIndex(WheelIdx);
		PVehicle->Suspension[WheelIdx].SetSpringIndex(WheelIdx);
		PVehicle->NumDrivenWheels = NumDrivenWheels;

		PVehicle->bLegacyWheelFrictionPosition = bLegacyWheelFrictionPosition;
	}

	RecalculateAxles();

	TransmissionSetup.SetTransferCaseModifier(transferCaseConfig.GetTransferCaseRatio());

	// setup axles in PVehicle
	for (auto& Axle : AxleToWheelMap)
	{
		TArray<int>& WheelIndices = Axle.Value;

		FDynamicWheelSetup& WheelSetup = (useSystem1ForDifferential?(WheelSetupsForDifferentialSystem1):(WheelSetupsForDifferentialSystem2))[WheelIndices[0]];
		UDynamicWheel* WheelData = WheelSetup.WheelClass.GetDefaultObject();

		Chaos::FAxleSim AxleSim;
		AxleSim.Setup.RollbarScaling = WheelData->RollbarScaling;

		for (int WheelIdx : WheelIndices)
		{
			AxleSim.Setup.WheelIndex.Add(WheelIdx);
		}

		PVehicle->Axles.Add(AxleSim);

	}

	// cache this value as it's useful for steering setup calculations and debug rendering
	WheelTrackDimensions = CalculateWheelLayoutDimensions();

	if (EngineSetup.TorqueCurve.GetRichCurve()->IsEmpty())
	{
		FString ActorName = "Unknown";
		if (GetOwner())
		{
			ActorName = GetOwner()->GetName();
		}
		UE_LOG(LogTemp, Warning, TEXT("Vehicle %s has no torque curve defined, disabling mechanical simulation."), *ActorName);

		bMechanicalSimEnabled = false;
	}

	if (bMechanicalSimEnabled)
	{
		Chaos::FSimpleEngineSim EngineSim(&EngineSetup.GetPhysicsEngineConfig());
		PVehicle->Engine.Add(EngineSim);

		Chaos::FSimpleTransmissionSim TransmissionSim(&TransmissionSetup.GetPhysicsTransmissionConfig());
		PVehicle->Transmission.Add(TransmissionSim);
		TransmissionType = TransmissionSim.Setup().TransmissionType; // current transmission mode - dynamically modifiable at runtime

		Chaos::FSimpleDifferentialSim DifferentialSim(&DifferentialSetup.GetPhysicsDifferentialConfig(useSystem1ForDifferential));
		PVehicle->Differential.Add(DifferentialSim);

		// Setup override of wheel TorqueRatio & EngineEnabled from vehicle differential settings
		for (int WheelIdx = 0; WheelIdx < PVehicle->Wheels.Num(); WheelIdx++)
		{
			FSimpleWheelSim& PWheel = PVehicle->Wheels[WheelIdx];
			bool IsWheelPowered = FTransmissionUtility::IsWheelPowered(DifferentialSim.Setup().DifferentialType, PWheel.Setup().AxleType, PWheel.EngineEnabled);
			PWheel.AccessSetup().EngineEnabled = IsWheelPowered;
			PWheel.EngineEnabled = IsWheelPowered;

			float TorqueRatio = FTransmissionUtility::GetTorqueRatioForWheel(DifferentialSim, WheelIdx, PVehicle->Wheels);
			PWheel.AccessSetup().TorqueRatio = TorqueRatio;
			
			//Use Override steering angle
			if (SteeringSetup.useOverrideSteerAngle)
			{
				if (PWheel.SteeringEnabled)
				{
					PWheel.MaxSteeringAngle =  SteeringSetup.overrideSteerAngle;
				}
			}
		}
	}

	Chaos::FSimpleSteeringSim SteeringSim(&SteeringSetup.GetPhysicsSteeringConfig(WheelTrackDimensions));
	PVehicle->Steering.Add(SteeringSim);

	Chaos::FTorqueControlSim TorqueSim(&TorqueControl.GetTorqueControlConfig());
	PVehicle->TorqueControlSim.Add(TorqueSim);

	Chaos::FTargetRotationControlSim TargetRotationSim(&TargetRotationControl.GetTargetRotationControlConfig());
	PVehicle->TargetRotationControlSim.Add(TargetRotationSim);

	Chaos::FStabilizeControlSim StabilizeSim(&StabilizeControl.GetStabilizeControlConfig());
	PVehicle->StabilizeControlSim.Add(StabilizeSim);

	// Setup the chassis and wheel shapes
	SetupVehicleShapes();

	// Setup mass properties
	SetupVehicleMass();

	// Setup Suspension
	SetupSuspension(PVehicle);
}

void UDynamicVehicleMovementComponent::ResetVehicleState()
{
	UChaosVehicleMovementComponent::ResetVehicleState();

	for (FDynamicWheelStatus& WheelInfo : WheelStatus)
	{
		WheelInfo.Init();
	}

	for (FCachedState& State : CachedState)
	{
		State.bIsValid = false;
	}
}

EDynamicDifferentialModes UDynamicVehicleMovementComponent::GetCurrentDifferentialMode()
{
	return currentDifferentialMode;
}


EDynamicVehicleDifferential UDynamicVehicleMovementComponent::GetCurrentDifferentialSystem()
{
	return IsUsingSystem1ForDifferential() ? DifferentialSetup.DifferentialTypeForSystem1 : DifferentialSetup.DifferentialTypeForSystem2;
}

void UDynamicVehicleMovementComponent::SetupVehicleShapes()
{
	if (!UpdatedPrimitive)
	{
		return;
	}

}

void UDynamicVehicleMovementComponent::SetupSuspension(TUniquePtr<Chaos::FSimpleWheeledVehicle>& PVehicle)
{
	if (!PVehicle.IsValid() || PVehicle->Suspension.Num() == 0)
	{
		return;
	}

	float TotalMass = this->Mass;
	ensureMsgf(TotalMass >= 1.0f, TEXT("The mass of this vehicle is too small."));

	TArray<FVector> LocalSpringPositions;

	// cache vehicle local position of springs
	for (int SpringIdx = 0; SpringIdx < PVehicle->Suspension.Num(); SpringIdx++)
	{
		auto& PSuspension = PVehicle->Suspension[SpringIdx];

		PSuspension.AccessSetup().MaxLength = PSuspension.Setup().SuspensionMaxDrop + PSuspension.Setup().SuspensionMaxRaise;

		FVector TotalOffset = GetWheelRestingPosition((useSystem1ForDifferential?(WheelSetupsForDifferentialSystem1):(WheelSetupsForDifferentialSystem2))[SpringIdx]);
		LocalSpringPositions.Add(TotalOffset);
		PVehicle->Suspension[SpringIdx].SetLocalRestingPosition(LocalSpringPositions[SpringIdx]);
	}

	// Calculate the mass that will rest on each of the springs
	TArray<float> OutSprungMasses;
	if (!LocalSpringPositions.IsEmpty() && !FSuspensionUtility::ComputeSprungMasses(LocalSpringPositions, TotalMass, OutSprungMasses))
	{
		// if the sprung mass calc fails fall back to something that will still simulate
		for (int Index = 0; Index < OutSprungMasses.Num(); Index++)
		{
			OutSprungMasses[Index] = TotalMass / OutSprungMasses.Num();
		}
	}

	// Calculate spring damping values we will use for physics simulation from the normalized damping ratio
	for (int SpringIdx = 0; SpringIdx < PVehicle->Suspension.Num(); SpringIdx++)
	{
		auto& Susp = PVehicle->Suspension[SpringIdx];
		float NaturalFrequency = FSuspensionUtility::ComputeNaturalFrequency(Susp.Setup().SpringRate, OutSprungMasses[SpringIdx]);
		float Damping = FSuspensionUtility::ComputeDamping(Susp.Setup().SpringRate, OutSprungMasses[SpringIdx], Susp.Setup().DampingRatio);
		UE_LOG(LogChaos, Verbose, TEXT("Spring %d: OutNaturalFrequency %.1f Hz  (@1.0) DampingRate %.1f"), SpringIdx, NaturalFrequency / (2.0f * PI), Damping);

		PVehicle->Suspension[SpringIdx].AccessSetup().ReboundDamping = Damping;
		PVehicle->Suspension[SpringIdx].AccessSetup().CompressionDamping = Damping;
		PVehicle->Suspension[SpringIdx].AccessSetup().RestingForce = OutSprungMasses[SpringIdx] * -GetGravityZ();
	}

}

void UDynamicVehicleMovementComponent::RecalculateAxles()
{
	AxleToWheelMap.Empty();
	for (int WheelIdx = 0; WheelIdx < (useSystem1ForDifferential?(WheelSetupsForDifferentialSystem1):(WheelSetupsForDifferentialSystem2)).Num(); WheelIdx++)
	{
		FDynamicWheelSetup& WheelSetup = (useSystem1ForDifferential?(WheelSetupsForDifferentialSystem1):(WheelSetupsForDifferentialSystem2))[WheelIdx];

		UDynamicWheel* Wheel = WheelSetup.WheelClass.GetDefaultObject();

		if (auto* WheelIdxArray = AxleToWheelMap.Find(Wheel))
		{
			WheelIdxArray->Add(WheelIdx);
		}
		else
		{
			TArray<int> WheelIndices;
			WheelIndices.Add(WheelIdx);
			AxleToWheelMap.Add(Wheel, WheelIndices);
		}

	}
}

FVector UDynamicVehicleMovementComponent::GetWheelRestingPosition(const FDynamicWheelSetup& WheelSetup)
{
	FVector Offset = WheelSetup.WheelClass.GetDefaultObject()->Offset + WheelSetup.AdditionalOffset;
	return LocateBoneOffset(WheelSetup.BoneName, Offset);
}

float UDynamicVehicleMovementComponent::GetEngineRotationSpeed() const
{
	float EngineRPM = 0.f;

	if (bMechanicalSimEnabled && PVehicleOutput)
	{
		EngineRPM = PVehicleOutput->EngineRPM;

		if (!IsEngineStarted())
		{
			EngineRPM = 0;
		}
		if (useDelayedRPM)
		{
			EngineRPM = fakeRPM;

		}
	}

	return EngineRPM;
}

float UDynamicVehicleMovementComponent::GetEngineMaxRotationSpeed() const
{
	float MaxEngineRPM = 0.f;

	if (bMechanicalSimEnabled)
	{
		MaxEngineRPM = EngineSetup.MaxRPM;
	}

	return MaxEngineRPM;
}



// Helper
FVector2D UDynamicVehicleMovementComponent::CalculateWheelLayoutDimensions()
{
	FVector2D MaxSize(0.f, 0.f);

	for (int32 WheelIdx = 0; WheelIdx < (useSystem1ForDifferential?(WheelSetupsForDifferentialSystem1):(WheelSetupsForDifferentialSystem2)).Num(); ++WheelIdx)
	{
		FDynamicWheelSetup& WheelSetup = (useSystem1ForDifferential?(WheelSetupsForDifferentialSystem1):(WheelSetupsForDifferentialSystem2))[WheelIdx];
		UDynamicWheel* Wheel = WheelSetup.WheelClass.GetDefaultObject();
		check(Wheel);

		const FVector WheelOffset = GetWheelRestingPosition(WheelSetup);
		if (FMath::Abs(WheelOffset.Y) > MaxSize.Y)
		{
			MaxSize.Y = FMath::Abs(WheelOffset.Y);
		}

		if (FMath::Abs(WheelOffset.X) > MaxSize.X)
		{
			MaxSize.X = FMath::Abs(WheelOffset.X);
		}

	}

	// full width/length not half
	MaxSize *= 2.0f;

	return MaxSize;
}

void UDynamicVehicleMovementComponent::Update(float DeltaTime)
{
	UChaosVehicleMovementComponent::Update(DeltaTime);

	if (CurAsyncInput)
	{
		if (const FBodyInstance* BodyInstance = GetBodyInstance())
		{
			if (auto Handle = BodyInstance->ActorHandle)
			{
				FChaosVehicleAsyncInput* AsyncInput = static_cast<FChaosVehicleAsyncInput*>(CurAsyncInput);

				TArray<AActor*> ActorsToIgnore;
				ActorsToIgnore.Add(GetPawnOwner()); // ignore self in scene query

				FCollisionQueryParams TraceParams(NAME_None, FCollisionQueryParams::GetUnknownStatId(), false, nullptr);
				TraceParams.bReturnPhysicalMaterial = true;	// we need this to get the surface friction coefficient
				TraceParams.AddIgnoredActors(ActorsToIgnore);
				TraceParams.bTraceComplex = true;
				AsyncInput->PhysicsInputs.TraceParams = TraceParams;
				AsyncInput->PhysicsInputs.TraceCollisionResponse = WheelTraceCollisionResponses;

				AsyncInput->PhysicsInputs.WheelTraceParams.SetNum(Wheels.Num());
				for (int I = 0; I < Wheels.Num(); I++)
				{
					AsyncInput->PhysicsInputs.WheelTraceParams[I].SweepType = Wheels[I]->SweepType;
					AsyncInput->PhysicsInputs.WheelTraceParams[I].SweepShape = Wheels[I]->SweepShape;
				}
			}
		}
	}
}

#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)

float UDynamicVehicleMovementComponent::CalcDialAngle(float CurrentValue, float MaxValue)
{
	return (CurrentValue / MaxValue) * 3.f / 2.f * PI - (PI * 0.25f);
}

void UDynamicVehicleMovementComponent::DrawDial(UCanvas* Canvas, FVector2D Pos, float Radius, float CurrentValue, float MaxValue)
{
	float Angle = CalcDialAngle(CurrentValue, MaxValue);
	FVector2D PtEnd(Pos.X - FMath::Cos(Angle) * Radius, Pos.Y - FMath::Sin(Angle) * Radius);
	DrawLine2D(Canvas, Pos, PtEnd, FColor::White, 3.f);

	for (float I = 0; I < MaxValue; I += 1000.0f)
	{
		Angle = CalcDialAngle(I, MaxValue);
		PtEnd.Set(-FMath::Cos(Angle) * Radius, -FMath::Sin(Angle) * Radius);
		FVector2D PtStart = PtEnd * 0.8f;
		DrawLine2D(Canvas, Pos + PtStart, Pos + PtEnd, FColor::White, 2.f);
	}

	// the last checkmark
	Angle = CalcDialAngle(MaxValue, MaxValue);
	PtEnd.Set(-FMath::Cos(Angle) * Radius, -FMath::Sin(Angle) * Radius);
	FVector2D PtStart = PtEnd * 0.8f;
	DrawLine2D(Canvas, Pos + PtStart, Pos + PtEnd, FColor::Red, 2.f);

}

#endif

void UDynamicVehicleMovementComponent::FillWheelOutputState()
{
	if (const FChaosVehicleAsyncOutput* CurrentOutput = static_cast<FChaosVehicleAsyncOutput*>(CurAsyncOutput))
	{
		if (CurrentOutput->bValid && PVehicleOutput)
		{
			for (int WheelIdx = 0; WheelIdx < WheelStatus.Num(); WheelIdx++)
			{
				auto& PWheel = PVehicleOutput->Wheels[WheelIdx];

				FDynamicWheelStatus& State = WheelStatus[WheelIdx];

				State.bIsValid = true;
				State.bInContact = PWheel.bBlockingHit;
				State.ContactPoint = PWheel.ImpactPoint;
				State.HitLocation = PWheel.HitLocation;
				State.PhysMaterial = PWheel.PhysMaterial;
				State.NormalizedSuspensionLength = PWheel.NormalizedSuspensionLength;
				State.SpringForce = PWheel.SpringForce;
				State.SlipAngle = PWheel.SlipAngle;
				State.bIsSlipping = PWheel.bIsSlipping;
				State.SlipMagnitude = PWheel.SlipMagnitude;
				State.bIsSkidding = PWheel.bIsSkidding;
				State.SkidMagnitude = PWheel.SkidMagnitude;
				if (State.bIsSkidding)
				{
					State.SkidNormal = PWheel.SkidNormal;
					//DrawDebugLine(GetWorld()
					//	, State.ContactPoint
					//	, State.ContactPoint + State.SkidNormal
					//	, FColor::Yellow, true, -1.0f, 0, 4);
				}
				else
				{
					State.SkidNormal = FVector::ZeroVector;
				}
				State.DriveTorque = PWheel.DriveTorque;
				State.BrakeTorque = PWheel.BrakeTorque;
				State.bABSActivated = PWheel.bABSActivated;
			}
		}
	}
}

void UDynamicVehicleMovementComponent::BreakWheelStatus(const struct FDynamicWheelStatus& Status, bool& bInContact, FVector& ContactPoint, UPhysicalMaterial*& PhysMaterial, float& NormalizedSuspensionLength, float& SpringForce, float& SlipAngle, bool& bIsSlipping, float& SlipMagnitude, bool& bIsSkidding, float& SkidMagnitude, FVector& SkidNormal, float& DriveTorque, float& BrakeTorque, bool& bABSActivated)
{
	bInContact = Status.bInContact;
	ContactPoint = Status.ContactPoint;
	PhysMaterial = Status.PhysMaterial.Get();
	NormalizedSuspensionLength = Status.NormalizedSuspensionLength;
	SpringForce = Status.SpringForce;
	SlipAngle = Status.SlipAngle;
	bIsSlipping = Status.bIsSlipping;
	SlipMagnitude = Status.SlipMagnitude;
	bIsSkidding = Status.bIsSkidding;
	SkidMagnitude = Status.SkidMagnitude;
	SkidNormal = Status.SkidNormal;
	DriveTorque = Status.DriveTorque;
	BrakeTorque = Status.BrakeTorque;
	bABSActivated = Status.bABSActivated;
}

FDynamicWheelStatus UDynamicVehicleMovementComponent::MakeWheelStatus(bool bInContact, FVector& ContactPoint, UPhysicalMaterial* PhysMaterial, float NormalizedSuspensionLength, float SpringForce, float SlipAngle, bool bIsSlipping, float SlipMagnitude, bool bIsSkidding, float SkidMagnitude, FVector& SkidNormal, float DriveTorque, float BrakeTorque, bool bABSActivated)
{
	FDynamicWheelStatus Status;
	Status.bInContact = bInContact;
	Status.ContactPoint = ContactPoint;
	Status.PhysMaterial = PhysMaterial;
	Status.NormalizedSuspensionLength = NormalizedSuspensionLength;
	Status.SpringForce = SpringForce;
	Status.SlipAngle = SlipAngle;
	Status.bIsSlipping = bIsSlipping;
	Status.SlipMagnitude = SlipMagnitude;
	Status.bIsSkidding = bIsSkidding;
	Status.SkidMagnitude = SkidMagnitude;
	Status.SkidNormal = SkidNormal;
	Status.DriveTorque = DriveTorque;
	Status.BrakeTorque = BrakeTorque;
	Status.bABSActivated = bABSActivated;

	return Status;
}

void UDynamicVehicleMovementComponent::BreakWheeledSnapshot(const struct FWheeledSnaphotData& Snapshot, FTransform& Transform, FVector& LinearVelocity, FVector& AngularVelocity, int& SelectedGear, float& EngineRPM, TArray<FWheelSnapshot>& WheelSnapshots)
{
	Transform = Snapshot.Transform;
	LinearVelocity = Snapshot.LinearVelocity;
	AngularVelocity = Snapshot.AngularVelocity;
	SelectedGear = Snapshot.SelectedGear;
	EngineRPM = Snapshot.EngineRPM;
	WheelSnapshots = Snapshot.WheelSnapshots;
}

FWheeledSnaphotData UDynamicVehicleMovementComponent::MakeWheeledSnapshot(FTransform Transform, FVector LinearVelocity, FVector AngularVelocity, int SelectedGear, float EngineRPM, const TArray<FWheelSnapshot>& WheelSnapshots)
{
	FWheeledSnaphotData Snapshot;
	Snapshot.Transform = Transform;
	Snapshot.LinearVelocity = LinearVelocity;
	Snapshot.AngularVelocity = AngularVelocity;
	Snapshot.SelectedGear = SelectedGear;
	Snapshot.EngineRPM = EngineRPM;
	Snapshot.WheelSnapshots = WheelSnapshots;

	return Snapshot;
}

void UDynamicVehicleMovementComponent::BreakWheelSnapshot(const struct FWheelSnapshot& Snapshot, float& SuspensionOffset, float& WheelRotationAngle, float& SteeringAngle, float& WheelRadius, float& WheelAngularVelocity)
{
	SuspensionOffset = Snapshot.SuspensionOffset;
	WheelRotationAngle = Snapshot.WheelRotationAngle;
	SteeringAngle = Snapshot.SteeringAngle;
	WheelRadius = Snapshot.WheelRadius;
	WheelAngularVelocity = Snapshot.WheelAngularVelocity;
}

FWheelSnapshot UDynamicVehicleMovementComponent::MakeWheelSnapshot(float SuspensionOffset, float WheelRotationAngle, float SteeringAngle, float WheelRadius, float WheelAngularVelocity)
{
	FWheelSnapshot Snapshot;
	Snapshot.SuspensionOffset = SuspensionOffset;
	Snapshot.WheelRotationAngle = WheelRotationAngle;
	Snapshot.SteeringAngle = SteeringAngle;
	Snapshot.WheelRadius = WheelRadius;
	Snapshot.WheelAngularVelocity = WheelAngularVelocity;

	return Snapshot;
}

void UDynamicVehicleMovementComponent::ParallelUpdate(float DeltaSeconds)
{
	UChaosVehicleMovementComponent::ParallelUpdate(DeltaSeconds); \

		FillWheelOutputState(); // exposes wheel/suspension data to blueprint
}

void UDynamicVehicleMovementComponent::SetWheelClass(int WheelIndex, TSubclassOf<UDynamicWheel> InWheelClass)
{
	if (UpdatedPrimitive && InWheelClass)
	{
		FBodyInstance* TargetInstance = GetBodyInstance();

		if (TargetInstance && WheelIndex < Wheels.Num())
		{

			FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis)
				{
					if (VehicleSimulationPT)
					{
						UDynamicWheel* OldWheel = Wheels[WheelIndex];
						UDynamicWheel* NewWheel = NewObject<UDynamicWheel>(this, InWheelClass);
						NewWheel->Init(this, WheelIndex);

						VehicleSimulationPT->InitializeWheel(WheelIndex, &NewWheel->GetPhysicsWheelConfig());
						VehicleSimulationPT->InitializeSuspension(WheelIndex, &NewWheel->GetPhysicsSuspensionConfig());

						Wheels[WheelIndex] = NewWheel;

						OldWheel->Shutdown();
					}
				});


		}
	}

}

FWheeledSnaphotData UDynamicVehicleMovementComponent::GetSnapshot() const
{
	FWheeledSnaphotData WheelSnapshotData;
	UChaosVehicleMovementComponent::GetBaseSnapshot(WheelSnapshotData);

	WheelSnapshotData.EngineRPM = PVehicleOutput->EngineRPM;
	WheelSnapshotData.SelectedGear = PVehicleOutput->CurrentGear;
	WheelSnapshotData.WheelSnapshots.SetNum(Wheels.Num());

	int WheelIdx = 0;
	for (const UDynamicWheel* Wheel : Wheels)
	{
		WheelSnapshotData.WheelSnapshots[WheelIdx].SteeringAngle = Wheel->GetSteerAngle();
		WheelSnapshotData.WheelSnapshots[WheelIdx].SuspensionOffset = Wheel->GetSuspensionOffset();
		WheelSnapshotData.WheelSnapshots[WheelIdx].WheelRotationAngle = Wheel->GetRotationAngle();
		WheelSnapshotData.WheelSnapshots[WheelIdx].WheelRadius = Wheel->GetWheelRadius();
		WheelSnapshotData.WheelSnapshots[WheelIdx].WheelAngularVelocity = Wheel->GetWheelAngularVelocity();

		WheelIdx++;
	}

	return WheelSnapshotData;
}

void UDynamicVehicleMovementComponent::SetSnapshot(const FWheeledSnaphotData& SnapshotIn)
{
	UChaosVehicleMovementComponent::SetBaseSnapshot(SnapshotIn);

	FBodyInstance* TargetInstance = GetBodyInstance();
	if (TargetInstance)
	{
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis)
			{
				const FWheeledSnaphotData* WheelSnapshotData = static_cast<const FWheeledSnaphotData*>(&SnapshotIn);

				ensure(WheelSnapshotData->WheelSnapshots.Num() == VehicleSimulationPT->PVehicle->Wheels.Num());
				if (VehicleSimulationPT && VehicleSimulationPT->PVehicle)
				{
					for (int WheelIdx = 0; WheelIdx < WheelSnapshotData->WheelSnapshots.Num(); WheelIdx++)
					{
						ensure(VehicleSimulationPT->PVehicle->Wheels.Num() == VehicleSimulationPT->PVehicle->Suspension.Num());

						if (WheelIdx < VehicleSimulationPT->PVehicle->Wheels.Num())
						{
							const FWheelSnapshot& Data = WheelSnapshotData->WheelSnapshots[WheelIdx];
							Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIdx];
							Chaos::FSimpleSuspensionSim& VehicleSuspension = VehicleSimulationPT->PVehicle->Suspension[WheelIdx];

							VehicleWheel.SetSteeringAngle(Data.SteeringAngle);
							VehicleSuspension.SetSuspensionLength(Data.SuspensionOffset, Data.WheelRadius);
							VehicleWheel.SetAngularPosition(FMath::DegreesToRadians(-Data.WheelRotationAngle));
							VehicleWheel.SetWheelRadius(Data.WheelRadius);
							VehicleWheel.SetAngularVelocity(Data.WheelAngularVelocity);
						}
					}
				}
			});
	}
}

void UDynamicVehicleMovementComponent::SetMaxEngineTorque(float Torque)
{
	if (FBodyInstance* TargetInstance = GetBodyInstance())
	{
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis)
			{
				if (VehicleSimulationPT && VehicleSimulationPT->PVehicle)
				{
					VehicleSimulationPT->PVehicle->GetEngine().SetMaxTorque(Torque);
				}
			});
	}
}

void UDynamicVehicleMovementComponent::SetDragCoefficient(float DragCoeff)
{
	if (FBodyInstance* TargetInstance = GetBodyInstance())
	{
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis)
			{
				if (VehicleSimulationPT && VehicleSimulationPT->PVehicle)
				{
					VehicleSimulationPT->PVehicle->GetAerodynamics().SetDragCoefficient(DragCoeff);
				}
			});
	}
}

void UDynamicVehicleMovementComponent::SetDownforceCoefficient(float DownforceCoeff)
{
	if (FBodyInstance* TargetInstance = GetBodyInstance())
	{
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis)
			{
				if (VehicleSimulationPT && VehicleSimulationPT->PVehicle)
				{
					VehicleSimulationPT->PVehicle->GetAerodynamics().SetDownforceCoefficient(DownforceCoeff);
				}
			});
	}
}

void UDynamicVehicleMovementComponent::SetDifferentialFrontRearSplit(float FrontRearSplit)
{
	if (FBodyInstance* TargetInstance = GetBodyInstance())
	{
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis)
			{
				if (VehicleSimulationPT && VehicleSimulationPT->PVehicle)
				{
					VehicleSimulationPT->PVehicle->GetDifferential().FrontRearSplit = FrontRearSplit;
				}
			});
	}
}

void UDynamicVehicleMovementComponent::SetTractionControlEnabled(int WheelIndex, bool Enabled)
{
	if (FBodyInstance* TargetInstance = GetBodyInstance())
	{
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis)
			{
				if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num())
				{
					Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

					VehicleWheel.TractionControlEnabled = Enabled;
				}
			});
	}
}

void UDynamicVehicleMovementComponent::SetABSEnabled(int WheelIndex, bool Enabled)
{
	if (FBodyInstance* TargetInstance = GetBodyInstance())
	{
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis)
			{
				if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num())
				{
					Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

					VehicleWheel.ABSEnabled = Enabled;
				}
			});
	}
}

void UDynamicVehicleMovementComponent::SetAffectedByBrake(int WheelIndex, bool Enabled)
{
	if (FBodyInstance* TargetInstance = GetBodyInstance())
	{
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis)
			{
				if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num())
				{
					Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

					VehicleWheel.BrakeEnabled = Enabled;
				}
			});
	}
}

void UDynamicVehicleMovementComponent::SetAffectedByHandbrake(int WheelIndex, bool Enabled)
{
	if (FBodyInstance* TargetInstance = GetBodyInstance())
	{
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis)
			{
				if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num())
				{
					Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

					VehicleWheel.HandbrakeEnabled = Enabled; // this is affecting all vehicles - needs to be per instance 
				}
			});
	}
}

void UDynamicVehicleMovementComponent::SetAffectedBySteering(int WheelIndex, bool Enabled)
{
	if (FBodyInstance* TargetInstance = GetBodyInstance())
	{
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis)
			{
				if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num())
				{
					Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

					VehicleWheel.SteeringEnabled = Enabled;
				}
			});
	}
}


void UDynamicVehicleMovementComponent::SetAffectedByEngine(int WheelIndex, bool Enabled)
{
	if (FBodyInstance* TargetInstance = GetBodyInstance())
	{
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis)
			{
				if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num())
				{
					Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

					VehicleWheel.EngineEnabled = Enabled;
				}
			});
	}

}

void UDynamicVehicleMovementComponent::SetWheelRadius(int WheelIndex, float Radius)
{
	if (FBodyInstance* TargetInstance = GetBodyInstance())
	{
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis)
			{
				if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num())
				{
					Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

					VehicleWheel.SetWheelRadius(Radius);
				}
			});
	}
}

void UDynamicVehicleMovementComponent::SetWheelFrictionMultiplier(int WheelIndex, float Friction)
{
	if (FBodyInstance* TargetInstance = GetBodyInstance())
	{
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis)
			{
				if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num())
				{
					Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

					VehicleWheel.FrictionMultiplier = Friction;
				}
			});
	}
}

void UDynamicVehicleMovementComponent::SetWheelSlipGraphMultiplier(int WheelIndex, float Multiplier)
{
	if (FBodyInstance* TargetInstance = GetBodyInstance())
	{
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis)
			{
				if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num())
				{
					Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

					VehicleWheel.LateralSlipGraphMultiplier = Multiplier;
				}
			});
	}
}

void UDynamicVehicleMovementComponent::SetWheelMaxBrakeTorque(int WheelIndex, float Torque)
{
	if (FBodyInstance* TargetInstance = GetBodyInstance())
	{
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis)
			{
				if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num())
				{
					Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

					VehicleWheel.MaxBrakeTorque = Torque;
				}
			});
	}
}

void UDynamicVehicleMovementComponent::SetWheelHandbrakeTorque(int WheelIndex, float Torque)
{
	if (FBodyInstance* TargetInstance = GetBodyInstance())
	{
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis)
			{
				if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num())
				{
					Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

					VehicleWheel.HandbrakeTorque = Torque;
				}
			});
	}
}

void UDynamicVehicleMovementComponent::SetWheelMaxSteerAngle(int WheelIndex, float AngleDegrees)
{
	if (FBodyInstance* TargetInstance = GetBodyInstance())
	{
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis)
			{
				if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num())
				{
					Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

					VehicleWheel.MaxSteeringAngle = AngleDegrees;
				}
			});
	}
}

void UDynamicVehicleMovementComponent::SetTorqueCombineMethod(ETorqueCombineMethod InCombineMethod, int32 WheelIndex)
{
	if (FBodyInstance* TargetInstance = GetBodyInstance())
	{
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis)
			{
				if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num())
				{
					Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

					VehicleWheel.SetTorqueCombineMethod(static_cast<Chaos::FSimpleWheelConfig::EExternalTorqueCombineMethod>(InCombineMethod));
				}
			});
	}
}

void UDynamicVehicleMovementComponent::SetDriveTorque(float DriveTorque, int32 WheelIndex)
{
	using namespace Chaos;

	SetSleeping(false);

	if (FBodyInstance* TargetInstance = GetBodyInstance())
	{
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis)
			{
				if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num())
				{
					Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

					VehicleWheel.SetDriveTorqueOverride(TorqueMToCm(DriveTorque));
				}
			});
	}
}

void UDynamicVehicleMovementComponent::SetBrakeTorque(float BrakeTorque, int32 WheelIndex)
{
	using namespace Chaos;

	if (FBodyInstance* TargetInstance = GetBodyInstance())
	{
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis)
			{
				if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num())
				{
					Chaos::FSimpleWheelSim& VehicleWheel = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

					VehicleWheel.SetBrakeTorqueOverride(TorqueMToCm(BrakeTorque));
				}
			});
	}
}

void UDynamicVehicleMovementComponent::SetSuspensionParams(float Rate, float Damping, float Preload, float MaxRaise, float MaxDrop, int32 WheelIndex)
{
	using namespace Chaos;

	if (FBodyInstance* TargetInstance = GetBodyInstance())
	{
		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis)
			{
				if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && WheelIndex < VehicleSimulationPT->PVehicle->Wheels.Num())
				{
					Chaos::FSimpleSuspensionSim& VehicleSuspension = VehicleSimulationPT->PVehicle->Suspension[WheelIndex];

					VehicleSuspension.AccessSetup().SpringRate = Rate;
					VehicleSuspension.AccessSetup().DampingRatio = Damping;
					VehicleSuspension.AccessSetup().SpringPreload = Preload;
					VehicleSuspension.AccessSetup().SetSuspensionMaxRaise(MaxRaise);
					VehicleSuspension.AccessSetup().SetSuspensionMaxDrop(MaxDrop);
				}
			});

		FPhysicsCommand::ExecuteWrite(TargetInstance->ActorHandle, [&](const FPhysicsActorHandle& Chassis)
			{
				if (WheelIndex < (useSystem1ForDifferential?(WheelSetupsForDifferentialSystem1):(WheelSetupsForDifferentialSystem2)).Num() && WheelIndex < Wheels.Num())
				{
					FDynamicWheelSetup& WheelSetup = (useSystem1ForDifferential?(WheelSetupsForDifferentialSystem1):(WheelSetupsForDifferentialSystem2))[WheelIndex];
					const FVector LocalWheel = GetWheelRestingPosition(WheelSetup);
					FPhysicsConstraintHandle ConstraintHandle = FPhysicsInterface::CreateSuspension(Chassis, LocalWheel);

					if (ConstraintHandle.IsValid())
					{
						UDynamicWheel* Wheel = Wheels[WheelIndex];
						check(Wheel);
						if (Chaos::FSuspensionConstraint* Constraint = static_cast<Chaos::FSuspensionConstraint*>(ConstraintHandles[WheelIndex].Constraint))
						{
							Constraint->SetHardstopStiffness(1.0f);
							Constraint->SetSpringStiffness(Chaos::MToCm(Rate) * 0.25f);
							Constraint->SetSpringPreload(Chaos::MToCm(Preload));
							Constraint->SetSpringDamping(Damping * 5.0f);
							Constraint->SetMinLength(-MaxRaise);
							Constraint->SetMaxLength(MaxDrop);
						}
					}
				}
			});

	}
}

float UDynamicVehicleMovementComponent::GetSuspensionOffset(int WheelIndex)
{
	float Offset = 0.f;

	auto CalcOffset = [this](ESweepShape& SweepShape, FVector& LocalHitPoint, FVector& LocalPos, float Radius)
		{
			float NewOffset = 0.0f;
			if (SweepShape == ESweepShape::Spherecast)
			{
				NewOffset = LocalHitPoint.Z - LocalPos.Z;
			}
			else
			{
				NewOffset = LocalHitPoint.Z - LocalPos.Z + Radius;
			}
			return NewOffset;
		};

	FDynamicWheelSetup& WheelSetup = (useSystem1ForDifferential?(WheelSetupsForDifferentialSystem1):(WheelSetupsForDifferentialSystem2))[WheelIndex];
	if (GetBodyInstance())
	{
		FTransform VehicleWorldTransform = GetBodyInstance()->GetUnrealWorldTransform();
		if (UDynamicWheel* Wheel = (useSystem1ForDifferential?(WheelSetupsForDifferentialSystem1):(WheelSetupsForDifferentialSystem2))[WheelIndex].WheelClass.GetDefaultObject())
		{
			if (WheelStatus[WheelIndex].bIsValid)
			{
				if (WheelStatus[WheelIndex].bInContact)
				{
					FVector LocalPos = GetWheelRestingPosition(WheelSetup);

					FVector ReferencePos = (Wheel->SweepShape == ESweepShape::Spherecast) ? WheelStatus[WheelIndex].HitLocation : WheelStatus[WheelIndex].ContactPoint;
					FVector LocalHitPoint = VehicleWorldTransform.InverseTransformPosition(ReferencePos);
					float Radius = PVehicleOutput->Wheels[WheelIndex].WheelRadius;

					if (CachedState[WheelIndex].bIsValid)
					{
						Offset = CachedState[WheelIndex].WheelOffset;

						float NewOffset = CalcOffset(Wheel->SweepShape, LocalHitPoint, LocalPos, Radius);

						// interpolate between old and new positions or will just jump to new position if Wheel->SuspensionSmoothing == 0
						float InterpolationMultiplier = 1.0f - (Wheel->SuspensionSmoothing / 11.0f);
						float Delta = NewOffset - CachedState[WheelIndex].WheelOffset;
						Offset += Delta * InterpolationMultiplier;
					}
					else
					{
						Offset = CalcOffset(Wheel->SweepShape, LocalHitPoint, LocalPos, Radius);
					}
					Offset = FMath::Clamp(Offset, -Wheel->SuspensionMaxDrop, Wheel->SuspensionMaxRaise);
				}
				else
				{
					Offset = -Wheel->SuspensionMaxDrop;
				}

				CachedState[WheelIndex].bIsValid = true;
				CachedState[WheelIndex].WheelOffset = Offset;

			}
			else
			{
				if (VehicleState.bSleeping && CachedState[WheelIndex].bIsValid)
				{
					Offset = CachedState[WheelIndex].WheelOffset;
				}
				else
				{
					ECollisionChannel SpringCollisionChannel = ECollisionChannel::ECC_WorldDynamic;
					FCollisionResponseParams ResponseParams;
					ResponseParams.CollisionResponse = WheelTraceCollisionResponses;

					TArray<AActor*> ActorsToIgnore;
					ActorsToIgnore.Add(GetPawnOwner()); // ignore self in scene query

					FCollisionQueryParams TraceParams(NAME_None, FCollisionQueryParams::GetUnknownStatId(), false, nullptr);
					TraceParams.bReturnPhysicalMaterial = true;	// we need this to get the surface friction coefficient
					TraceParams.AddIgnoredActors(ActorsToIgnore);
					TraceParams.bTraceComplex = (Wheels[WheelIndex]->SweepType == ESweepType::ComplexSweep);

					FVector LocalDirection = Wheel->SuspensionAxis;
					FVector WorldLocation = VehicleWorldTransform.TransformPosition(GetWheelRestingPosition(WheelSetup));
					FVector WorldDirection = VehicleWorldTransform.TransformVector(LocalDirection);

					FVector TraceStart = WorldLocation - WorldDirection * (Wheel->SuspensionMaxRaise);
					FVector TraceEnd = WorldLocation + WorldDirection * (Wheel->SuspensionMaxDrop + Wheel->WheelRadius);

					FHitResult HitResult;
					switch (Wheel->SweepShape)
					{
					case ESweepShape::Spherecast:
					{
						float WheelRadius = Wheel->WheelRadius;

						GetWorld()->SweepSingleByChannel(HitResult
							, TraceStart
							, TraceEnd
							, FQuat::Identity, SpringCollisionChannel
							, FCollisionShape::MakeSphere(WheelRadius), TraceParams
							, ResponseParams);
					}
					break;

					case ESweepShape::Raycast:
					default:
					{
						GetWorld()->LineTraceSingleByChannel(HitResult, TraceStart, TraceEnd, SpringCollisionChannel, TraceParams, ResponseParams);
					}
					break;
					}

					if (HitResult.bBlockingHit)
					{
						FVector LocalPos = GetWheelRestingPosition(WheelSetup);
						if (Wheel->SweepShape == ESweepShape::Spherecast)
						{
							FVector LocalHitPoint = VehicleWorldTransform.InverseTransformPosition(HitResult.Location);
							Offset = CalcOffset(Wheel->SweepShape, LocalHitPoint, LocalPos, Wheel->WheelRadius);
						}
						else
						{
							FVector LocalHitPoint = VehicleWorldTransform.InverseTransformPosition(HitResult.ImpactPoint);
							Offset = CalcOffset(Wheel->SweepShape, LocalHitPoint, LocalPos, Wheel->WheelRadius);
						}

						Offset = FMath::Clamp(Offset, -Wheel->SuspensionMaxDrop, Wheel->SuspensionMaxRaise);
					}
					else
					{
						Offset = -Wheel->SuspensionMaxDrop;
					}

					CachedState[WheelIndex].bIsValid = true;
					CachedState[WheelIndex].WheelOffset = Offset;
				}
			}
		}
	}
	return Offset;
}

UPhysicalMaterial* UDynamicVehicleMovementComponent::GetPhysMaterial(int WheelIndex)
{
	return WheelStatus[WheelIndex].PhysMaterial.Get();
}

FDynamicWheelSetup::FDynamicWheelSetup() 
{
	WheelClass = UDynamicWheel::StaticClass();
	BoneName = NAME_None;
	AdditionalOffset = FVector(0.0f);
}

//CUSTOMIZED FUNCTIONS START HERE

#if WITH_EDITOR

void UDynamicVehicleMovementComponent::PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent)
{
	const FName PropertyName = PropertyChangedEvent.Property ? PropertyChangedEvent.Property->GetFName() : NAME_None;

	RecalculateAxles();

	if ((TransmissionSetup.vehicleTransmissionType==ETransmissionType::Automatic || TransmissionSetup.vehicleTransmissionType == ETransmissionType::Hybrid) && PropertyName=="vehicleTransmissionType")		//Auto Vehicle 
	{
		//Set functionality defaults
		vehicleFunctionalities.vehicleHasHighLowGears = false; 
		vehicleFunctionalities.vehicleHasManualFuelHandle = false;
		fuelValueToSustainIdleRPM = 0;
		//vehicleFunctionalities.vehicleHasTransferCase = false; Vehicle can have changeable trasnfercase in automatic
		//vehicleFunctionalities.vehicleHasMultipleDifferentials = false; Possible in automatic
		

		//Set transmission setup values
		TransmissionSetup.bUseHighLowRatios = false;
	}
	else if (TransmissionSetup.vehicleTransmissionType == ETransmissionType::Manual && PropertyName == "vehicleTransmissionType")	//Manual Vehicle
	{
		bReverseAsBrake = false;
		bThrottleAsBrake = false;
	}

	if(vehicleFunctionalities.vehicleHasManualFuelHandle && PropertyName == "vehicleHasManualFuelHandle")
		fuelValueToSustainIdleRPM = 30;
	else if(PropertyName == "vehicleHasManualFuelHandle")
		fuelValueToSustainIdleRPM = gasPedalMinValue;

	if (vehicleFunctionalities.vehicleHasHighLowGears && PropertyName == "vehicleHasHighLowGears")
		TransmissionSetup.bUseHighLowRatios = true;
	else if (PropertyName == "vehicleHasHighLowGears")
		TransmissionSetup.bUseHighLowRatios = false;

	if (vehicleFunctionalities.vehicleHasTransferCase && PropertyName == "vehicleHasTransferCase")
		transferCaseConfig.isTransferCaseActive = true;
	else if (PropertyName == "vehicleHasTransferCase")
		transferCaseConfig.isTransferCaseActive = false;

	if (PropertyName == "vehicleTransmissionType")
	{
		if (TransmissionSetup.vehicleTransmissionType == ETransmissionType::Automatic)
		{
			for (int i = 0; i < TransmissionSetup.ForwardGearRatiosSingular.Num(); i++)
			{
				TransmissionSetup.ForwardGearRatiosSingular[i].useSpeedLimitations = false;
				TransmissionSetup.ForwardGearRatiosSingular[i].MinimumSpeed = 0;
				TransmissionSetup.ForwardGearRatiosSingular[i].MaximumSpeed = -1;
			}

			for (int i = 0; i < TransmissionSetup.ReverseGearRatiosSingular.Num(); i++)
			{
				TransmissionSetup.ReverseGearRatiosSingular[i].useSpeedLimitations = false;
				TransmissionSetup.ReverseGearRatiosSingular[i].MinimumSpeed = 0;
				TransmissionSetup.ReverseGearRatiosSingular[i].MaximumSpeed = -1;
			}
		}
	}

	if (PropertyName == "defaultDifferentialMode")
	{
		if (vehicleFunctionalities.vehicleHasDifferenitalModes)
		{
			EDynamicVehicleDifferential diffsystem = GetCurrentDifferentialSystem();

			if (diffsystem == EDynamicVehicleDifferential::AllWheelDrive)
			{
				if (currentDifferentialMode == EDynamicDifferentialModes::InterAxleLock || currentDifferentialMode == EDynamicDifferentialModes::AllDifferentials || currentDifferentialMode == EDynamicDifferentialModes::OpenDifferential || currentDifferentialMode == EDynamicDifferentialModes::RearDifferentialLock)
				{
					currentDifferentialMode = currentDifferentialMode;
				}
				else
					currentDifferentialMode = EDynamicDifferentialModes::OpenDifferential;

			}
			else if (diffsystem == EDynamicVehicleDifferential::RearWheelDrive)
			{
				if (currentDifferentialMode == EDynamicDifferentialModes::RearDifferentialLock || currentDifferentialMode == EDynamicDifferentialModes::OpenDifferential)
				{
					currentDifferentialMode = currentDifferentialMode;
				}
				else
					currentDifferentialMode = EDynamicDifferentialModes::OpenDifferential;

			}
			else if (diffsystem == EDynamicVehicleDifferential::FrontWheelDrive)
			{
				if (currentDifferentialMode == EDynamicDifferentialModes::OpenDifferential)
				{
					currentDifferentialMode = currentDifferentialMode;
				}
				else
					currentDifferentialMode = EDynamicDifferentialModes::OpenDifferential;

			}
		}
		else
			currentDifferentialMode = EDynamicDifferentialModes::OpenDifferential;
	}

	if (editDefaultRanges)
	{
		if (gasPedalMinValue >= gasPedalMaxValue)
		{
			gasPedalMaxValue = gasPedalMinValue + 1;
			UE_LOG(LogTemp, Warning, TEXT("Max value for Gas pedal Input can not be lower than minimum value. Setting Max Value to Min Value+1"));

		}
		if (breakPedalMinValue >= breakPedalMaxValue)
		{
			breakPedalMaxValue = breakPedalMinValue + 1;
			UE_LOG(LogTemp, Warning, TEXT("Max value for Break pedal Input can not be lower than minimum value. Setting Max Value to Min Value+1"));

		}
		if (steerWheelMinValue >= steerWheelMaxValue)
		{
			steerWheelMaxValue = steerWheelMinValue + 1;
			UE_LOG(LogTemp, Warning, TEXT("Max value for Steering Wheel Input can not be lower than minimum value. Setting Max Value to Min Value+1"));

		}
		if (clutchPedalMinValue >= clutchPedalMaxValue)
		{
			clutchPedalMaxValue = clutchPedalMinValue + 1;
			UE_LOG(LogTemp, Warning, TEXT("Max value for Clutch pedal Input can not be lower than minimum value. Setting Max Value to Min Value+1"));

		}
		if (fuelValueToSustainIdleRPM < gasPedalMinValue || fuelValueToSustainIdleRPM > gasPedalMaxValue)
		{
			fuelValueToSustainIdleRPM = gasPedalMinValue;
			UE_LOG(LogTemp, Warning, TEXT("Fuel Value to sustain idle rpm should be within gas pedal range. Setting to Min Value"));
		}
		if (clutchThresholdValue < clutchPedalMinValue || clutchThresholdValue > clutchPedalMaxValue)
		{
			clutchThresholdValue = clutchPedalMinValue + 1;
			UE_LOG(LogTemp, Warning, TEXT("Clutch Threshold Value should be within clutch pedal range. Setting to Min Value+1"));
		}
		if (clutchDisengagementValue < clutchPedalMinValue || clutchDisengagementValue > clutchPedalMaxValue)
		{
			clutchDisengagementValue = clutchPedalMinValue + 1;
			UE_LOG(LogTemp, Warning, TEXT("Clutch Threshold Value should be within clutch pedal range. Setting to Min Value+1"));
		}
	}

	TransmissionSetup.SetTransferCaseModifier(transferCaseConfig.GetTransferCaseRatio());
	
	Super::PostEditChangeProperty(PropertyChangedEvent);

}

bool UDynamicVehicleMovementComponent::CanEditChange(const FProperty* InProperty) const
{
	const bool ParentVal = Super::CanEditChange(InProperty);
	if (InProperty->GetFName() == GET_MEMBER_NAME_CHECKED(UDynamicVehicleMovementComponent, bReverseAsBrake))
	{
		return ParentVal && TransmissionSetup.vehicleTransmissionType == ETransmissionType::Automatic;
	}
	if (InProperty->GetFName() == GET_MEMBER_NAME_CHECKED(UDynamicVehicleMovementComponent, bThrottleAsBrake))
	{
		return ParentVal && TransmissionSetup.vehicleTransmissionType == ETransmissionType::Automatic;
	}
	if (InProperty->GetFName() == GET_MEMBER_NAME_CHECKED(UDynamicVehicleMovementComponent, fuelValueToSustainIdleRPM))
	{
		return ParentVal && vehicleFunctionalities.vehicleHasManualFuelHandle && TransmissionSetup.vehicleTransmissionType == ETransmissionType::Manual ;
	}
	if (InProperty->GetFName() == GET_MEMBER_NAME_CHECKED(UDynamicVehicleMovementComponent, transferCaseConfig))
	{
		return ParentVal && vehicleFunctionalities.vehicleHasTransferCase;
	}
	if (InProperty->GetFName() == GET_MEMBER_NAME_CHECKED(UDynamicVehicleMovementComponent, vehicleNonFunctionalAspects))
	{
		return ParentVal && vehicleFunctionalities.vehicleHasNonFunctionalAspects;
	}
	
	if (InProperty->GetFName() == "EngineRevUpMOI")
	{
		return ParentVal && (TransmissionSetup.vehicleTransmissionType == ETransmissionType::Automatic || TransmissionSetup.vehicleTransmissionType == ETransmissionType::Hybrid) &&!EngineSetup.RPM_DependsOnFuelInput;
	}
	if (InProperty->GetFName() == "EngineRevDownRate")
	{
		return ParentVal && (TransmissionSetup.vehicleTransmissionType==ETransmissionType::Automatic || TransmissionSetup.vehicleTransmissionType == ETransmissionType::Hybrid) && !EngineSetup.RPM_DependsOnFuelInput;
	}
	if (InProperty->GetFName() == "useSpeedLimitations")
	{
		return ParentVal && (TransmissionSetup.vehicleTransmissionType != ETransmissionType::Automatic);
	}
	if (InProperty->GetFName() == "RPM_IncreasRate")
	{
		return ParentVal && EngineSetup.RPM_DependsOnFuelInput;
	}
	
	if (InProperty->GetFName() == "clutchDisengagementValue")
	{
		return ParentVal && TransmissionSetup.vehicleTransmissionType == ETransmissionType::Manual;
	}
	if (InProperty->GetFName() == "clutchThresholdValue")
	{
		return ParentVal && TransmissionSetup.vehicleTransmissionType == ETransmissionType::Manual;
	}
	if (InProperty->GetFName() == "clutchPedalMinValue")
	{
		return ParentVal && TransmissionSetup.vehicleTransmissionType == ETransmissionType::Manual;
	}
	if (InProperty->GetFName() == "clutchPedalMaxValue")
	{
		return ParentVal && TransmissionSetup.vehicleTransmissionType == ETransmissionType::Manual;
	}

	if (InProperty->GetFName() == "breakValueToAllowModeChange")
	{
		return ParentVal && TransmissionSetup.vehicleTransmissionType != ETransmissionType::Manual;
	}

	if (InProperty->GetFName() == "turboModeDuration")
	{
		return ParentVal && vehicleFunctionalities.vehicleHasTurboMode;
	}
	if (InProperty->GetFName() == "turboModeFactor")
	{
		return ParentVal && vehicleFunctionalities.vehicleHasTurboMode;
	}
	if (InProperty->GetFName() == "maxAngleAllowedLowGears")
	{
		return ParentVal && TransmissionSetup.bUseHighLowRatios;
	}
	
	if (InProperty->GetFName() == "defaultDifferentialMode")
	{
		return ParentVal && vehicleFunctionalities.vehicleHasDifferenitalModes;
	}
	return ParentVal;
}
#endif

UDynamicVehicleMovementComponent::UDynamicVehicleMovementComponent(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{

	// default values setup
	PrimaryComponentTick.bCanEverTick = true;


	EngineSetup.InitDefaults();
	DifferentialSetup.InitDefaults();
	TransmissionSetup.InitDefaults();
	SteeringSetup.InitDefaults();

	// It's possible to switch whole systems off if they are not required
	bMechanicalSimEnabled = true;
	bSuspensionEnabled = true;
	bWheelFrictionEnabled = true;

	// new vehicles don't use legacy method where friction forces are applied at wheel rather than wheel contact point 
	bLegacyWheelFrictionPosition = false;

	WheelTraceCollisionResponses = FCollisionResponseContainer::GetDefaultResponseContainer();
	WheelTraceCollisionResponses.Vehicle = ECR_Ignore;
	

	if (vehicleLights.useVehicleLights)
	{
		vehicleLights.headLightLeft.OtherActor = GetOwner();
		vehicleLights.headLightRight.OtherActor = GetOwner();
		vehicleLights.fogLightLeft.OtherActor = GetOwner();
		vehicleLights.fogLightRight.OtherActor = GetOwner();
		vehicleLights.rearLightLeft.OtherActor = GetOwner();
		vehicleLights.rearLightRight.OtherActor = GetOwner();
		vehicleLights.brakeLightLeft.OtherActor = GetOwner();
		vehicleLights.brakeLightRight.OtherActor = GetOwner();
		vehicleLights.fogLightLeftRear.OtherActor = GetOwner();
		vehicleLights.fogLightRightRear.OtherActor = GetOwner();
		vehicleLights.turnLightLeft.OtherActor = GetOwner();
		vehicleLights.turnLightLeftRear.OtherActor = GetOwner();
		vehicleLights.turnLightRight.OtherActor = GetOwner();
		vehicleLights.turnLightRightRear.OtherActor = GetOwner();
	}

#if WITH_EDITOR
	UFunction* setGearFunc = StaticClass()->FindFunctionByName(FName("SetTargetGear"));
	if (setGearFunc != nullptr)
	{
		setGearFunc->SetMetaData(FName("Category"), TEXT("Game|Components|DynamicVehicleMovement"));
		//setGearFunc->FunctionFlags &= ~FUNC_BlueprintCallable;
	}

	UFunction* setAutomaticGear = StaticClass()->FindFunctionByName(FName("SetUseAutomaticGears"));
	if (setAutomaticGear != nullptr)
	{
		setAutomaticGear->SetMetaData(FName("Category"), TEXT("Game|Components|DynamicVehicleMovement"));
		setAutomaticGear->FunctionFlags &= ~FUNC_BlueprintCallable;
	}

	for (TFieldIterator<FProperty> propertyIterator(StaticClass()); propertyIterator; ++propertyIterator)
	{
		FProperty* Property = *propertyIterator;
		const FString& CurrentCategory = Property->GetMetaData(TEXT("Category"));

		if ("VehicleSetup" == FName(*CurrentCategory))
		{
			Property->SetMetaData(TEXT("Category"), "Dynamic Vehicle Movement|Core Setup|Vehicle Setup");
		}

	}
#endif
}

bool UDynamicVehicleMovementComponent::IsUsingSystem1ForDifferential() const
{
	return useSystem1ForDifferential;
}

bool UDynamicVehicleMovementComponent::SetActiveSystemForDifferential(bool UseSystem1, FString& failureReason)
{
	if (vehicleFunctionalities.vehicleHasMultipleDifferentials)
	{
		if (CanChangeDifferentialSystem())
		{

			if (UseSystem1 == useSystem1ForDifferential)
			{
				failureReason = "Trying to change to same differential system. ";
				return false;
			}
			else if (UseSystem1 == true)
			{
				useSystem1ForDifferential = true;
			}
			else if (UseSystem1 == false)
			{
				useSystem1ForDifferential = false;
			}

			CreateVehicle();
			FixupSkeletalMesh();
			if (derivedPtrForSimulationClass)
			{
				derivedPtrForSimulationClass->SetDifferentialSystem(useSystem1ForDifferential ? DifferentialSetup.DifferentialTypeForSystem1 : DifferentialSetup.DifferentialTypeForSystem2);
				derivedPtrForSimulationClass->dataImpactingSimulation.SetMaxGearLock(maxGearLock);

			}
			return true;
		}
		else
		{
			failureReason = "Not in rest and neutral position";
			return false;
		}
	}
	else
	{
		failureReason = "Vehicle does not have multiple differential systems";
		return false;
	}
}

bool UDynamicVehicleMovementComponent::SetTransferCasePosition(ETransferCasePosition newPosition, FString& failureReason)
{
	if (vehicleFunctionalities.vehicleHasTransferCase)
	{
		transferCaseConfig.isTransferCaseActive = true;
		if (CanChangeTransferCasePosition())
		{

			if (newPosition == transferCaseConfig.transferCasePosition)
			{
				failureReason = "Changing to same transfer case position";
			}
			else if (newPosition == ETransferCasePosition::HighRatio)
			{
				transferCaseConfig.transferCasePosition = ETransferCasePosition::HighRatio;
				currentTransferCaseRatio = transferCaseConfig.GetTransferCaseRatio();
				TransmissionSetup.SetTransferCaseModifier(currentTransferCaseRatio);
			}
			else if (newPosition == ETransferCasePosition::LowRatio)
			{
				transferCaseConfig.transferCasePosition = ETransferCasePosition::LowRatio;
				currentTransferCaseRatio = transferCaseConfig.GetTransferCaseRatio();
				TransmissionSetup.SetTransferCaseModifier(currentTransferCaseRatio);
			}
			else if (newPosition == ETransferCasePosition::Neutral)
			{
				transferCaseConfig.transferCasePosition = ETransferCasePosition::Neutral;
				currentTransferCaseRatio = transferCaseConfig.GetTransferCaseRatio();
				TransmissionSetup.SetTransferCaseModifier(currentTransferCaseRatio);
			}
			doOnceForLockedAxelError = true;
			CreateVehicle();
			FixupSkeletalMesh();

			if (derivedPtrForSimulationClass)
			{
				derivedPtrForSimulationClass->dataImpactingSimulation.SetMaxGearLock(maxGearLock);
				derivedPtrForSimulationClass->SetDifferentialSystem(GetCurrentDifferentialSystem());
			}

			return true;
		}
		else
		{
			failureReason = "Not in rest position";
			return false;
		}
	}
	else
	{
		transferCaseConfig.isTransferCaseActive = false;
		failureReason = "Vehicle does not have Transfer Case System";
		return false;
	}
}

bool UDynamicVehicleMovementComponent::SetTransferCasePositionUsingNum(int newPosition, FString& failureReason)
{
	ETransferCasePosition tempPosition;
	if (newPosition == -1)
		tempPosition = ETransferCasePosition::LowRatio;
	else if (newPosition == 0)
		tempPosition = ETransferCasePosition::Neutral;
	else if (newPosition == 1)
		tempPosition = ETransferCasePosition::HighRatio;
	else
	{
		failureReason = "Incorrect Numeral Identifier. Only -1, 0, 1 are possible!";
		return false;
	}

	return SetTransferCasePosition(tempPosition, failureReason);
}

bool UDynamicVehicleMovementComponent::CanChangeDifferentialSystem()
{
	if (vehicleFunctionalities.vehicleHasMultipleDifferentials)
	{
		float currentSpeed = GetVehicleSpeedInKM_PerHour();
		int currentGear = GetCurrentActiveGear();
		if (currentGear == 0 && (currentSpeed<1))
			return true;
		else
			return false;
	}
	else
		return false;
}

bool UDynamicVehicleMovementComponent::SetDifferentialMode(EDynamicDifferentialModes newMode)
{
	if (vehicleFunctionalities.vehicleHasDifferenitalModes)
	{
		EDynamicVehicleDifferential diffsystem = GetCurrentDifferentialSystem();

		if (diffsystem == EDynamicVehicleDifferential::AllWheelDrive)
		{
			if (newMode == EDynamicDifferentialModes::InterAxleLock || newMode == EDynamicDifferentialModes::AllDifferentials || newMode == EDynamicDifferentialModes::OpenDifferential || newMode == EDynamicDifferentialModes::RearDifferentialLock)
			{
				currentDifferentialMode = newMode;
			}
			else
				currentDifferentialMode = EDynamicDifferentialModes::OpenDifferential;

		}
		else if (diffsystem == EDynamicVehicleDifferential::RearWheelDrive)
		{
			if (newMode == EDynamicDifferentialModes::RearDifferentialLock || newMode == EDynamicDifferentialModes::OpenDifferential)
			{
				currentDifferentialMode = newMode;
			}
			else
				currentDifferentialMode = EDynamicDifferentialModes::OpenDifferential;

		}
		else if (diffsystem == EDynamicVehicleDifferential::FrontWheelDrive)
		{
			if (newMode == EDynamicDifferentialModes::OpenDifferential)
			{
				currentDifferentialMode = newMode;
			}
			else
				currentDifferentialMode = EDynamicDifferentialModes::OpenDifferential;

		}
	}
	else
		currentDifferentialMode = EDynamicDifferentialModes::OpenDifferential;

	if (derivedPtrForSimulationClass)
	{
		derivedPtrForSimulationClass->SetDifferentialMode(currentDifferentialMode);
	}
	return false;
}



bool UDynamicVehicleMovementComponent::CanChangeTransferCasePosition()
{
	if (vehicleFunctionalities.vehicleHasTransferCase)
	{
		float currentSpeed = GetVehicleSpeedInKM_PerHour();
		if (currentSpeed<1)
			return true;
		else
			return false;
	}
	else
		return false;
}

void UDynamicVehicleMovementComponent::GearUp()
{
	SetNewGear(GetCurrentActiveGear()+1);
}

void UDynamicVehicleMovementComponent::GearDown()
{
	SetNewGear(GetCurrentActiveGear() - 1);
}

void UDynamicVehicleMovementComponent::GoNeutral()
{
	if (TransmissionSetup.IsCurrentTransmissionModeManual())
	{
		SetNewGear(0);
		UpdateVehicleEngineState();
	}
	else
	{
		if (vehicleFunctionalities.autoVehicleNeedsBreakPress)
		{
			if (currentInputs.currentBreakPedalValue > (breakPedalMaxValue - breakPedalMinValue) * breakValueToAllowModeChange)
			{
				Super::SetTargetGear(0, true);
				UpdateVehicleEngineState();
			}
		}
		else
		{
			Super::SetTargetGear(0, true);
			UpdateVehicleEngineState();
		}
	}
	
}

void UDynamicVehicleMovementComponent::PutInReverse()
{
	if (IsEngineStarted())
	{
		if (TransmissionSetup.IsCurrentTransmissionModeManual())
			SetNewGear(-1);
		else
		{
			if (vehicleFunctionalities.autoVehicleNeedsBreakPress)
			{
				if (currentInputs.currentBreakPedalValue > (breakPedalMaxValue - breakPedalMinValue) * breakValueToAllowModeChange)
				{
					Super::SetTargetGear(-1, true);
				}
			}
			else
			{
				Super::SetTargetGear(-1, true);
			}
		}
	}
}

void UDynamicVehicleMovementComponent::SetDriveModeForAutomatic()
{
	if (IsEngineStarted())
	{
		if (vehicleFunctionalities.autoVehicleNeedsBreakPress)
		{
			if (currentInputs.currentBreakPedalValue > (breakPedalMaxValue - breakPedalMinValue) * breakValueToAllowModeChange)
			{
				if (!TransmissionSetup.IsCurrentTransmissionModeManual())
					Super::SetTargetGear(1, true);
			}
		}
		else
		{
			if (!TransmissionSetup.IsCurrentTransmissionModeManual())
				Super::SetTargetGear(1, true);
		}
	}
}

bool UDynamicVehicleMovementComponent::SetMaxGearLock(int GearNum)
{
	if(TransmissionSetup.vehicleTransmissionType == ETransmissionType::Manual && TransmissionSetup.bUseHighLowRatios )
		maxGearLock = FMath::Clamp(GearNum, 0, TransmissionSetup.ForwardGearRatios.Num());
	else
		maxGearLock = FMath::Clamp(GearNum, 0, TransmissionSetup.ForwardGearRatiosSingular.Num());

	if (derivedPtrForSimulationClass)
	{
		derivedPtrForSimulationClass->dataImpactingSimulation.SetMaxGearLock(maxGearLock);
	}
	if (GetCurrentActiveGear() > maxGearLock)
	{
		ChangeTransmissionSystem(false);
		GearDown();
		GetWorld()->GetTimerManager().SetTimer(gearDownToMaxGear_TimerHandle, this, &UDynamicVehicleMovementComponent::GearDownToMatchMaxGear, 0.5f, true);

	}

	return true;
	
}

void UDynamicVehicleMovementComponent::GearDownToMatchMaxGear()
{
	int currentGear = GetCurrentActiveGear();
	if (currentGear > maxGearLock)
	{
		GearDown();
	}
	if (currentGear == maxGearLock && GetEngineRotationSpeed() <= TransmissionSetup.ChangeUpRPM - 2) //allow to be called extra times to be safe
	{
		GetWorld()->GetTimerManager().ClearTimer(gearDownToMaxGear_TimerHandle);
		ChangeTransmissionSystem(true);
	}

}

int UDynamicVehicleMovementComponent::GetCurrentActiveGear()
{
	int activeGear;
	activeGear = GetCurrentGear();
	if (TransmissionSetup.IsCurrentTransmissionModeManual())
	{
		if (vehicleFunctionalities.vehicleHasHighLowGears)
		{
			if (activeGear > 0)
				activeGear = (activeGear + 1) / 2;
			else
				activeGear = (activeGear - 1) / 2;
		}
	}
	return activeGear;
}

float UDynamicVehicleMovementComponent::GetCurrentActiveGearRatio()
{
	if (!TransmissionSetup.IsCurrentTransmissionModeManual())
	{
		return TransmissionSetup.GetGearRatioAutomatic(GetCurrentActiveGear());
	}
	else
	{
		if (TransmissionSetup.bUseHighLowRatios)
		{
			FHighLowGearCombo combo = TransmissionSetup.GetGearRatio(GetCurrentActiveGear());
			return GetCurrentGear() % 2 == 0 ? (combo.LowRatio) : (combo.HighRatio);
		}
		else
		{
			FSingularGearCombo combo = TransmissionSetup.GetGearRatioSingular(GetCurrentActiveGear());
			return combo.RatioSingular;
		}
		
	}
	
}

float UDynamicVehicleMovementComponent::GetCurrentActiveGearRatioWithoutFinalGearRatioAffect()
{
	return (GetCurrentActiveGearRatio() / TransmissionSetup.FinalRatio);
}

bool UDynamicVehicleMovementComponent::IsUsingHighGears() const
{
	return usingHighGears&& vehicleFunctionalities.vehicleHasHighLowGears;
}

bool UDynamicVehicleMovementComponent::IsEngineStarted() const
{
	return currentEngineStartedValue;
}

bool UDynamicVehicleMovementComponent::IsVehicleCurrentlyAccelerating() const
{
	return isVehicleAccelerating;
}

void UDynamicVehicleMovementComponent::ChangeHighLowGearSystem(bool useHighGears = true)
{
	if (vehicleFunctionalities.vehicleHasHighLowGears)
	{
		usingHighGears = useHighGears;
		int currentGear = GetCurrentActiveGear();
		if (currentGear != 0)
		{
			if (SetNewGear(currentGear, true))
			{
				transmissionSystemChanged.Broadcast(usingHighGears);
			}
		}
	}
}

bool UDynamicVehicleMovementComponent::CanChangeGear()
{
	if (TransmissionSetup.vehicleTransmissionType == ETransmissionType::Manual)
	{
		return currentEngineState == EEngineState::EngineGearChangeable || currentEngineState == EEngineState::EngineDisengaged;
	}
	else
	{
		return TransmissionSetup.IsCurrentTransmissionModeManual();
	}
}

bool UDynamicVehicleMovementComponent::SetNewGear(int GearNum, bool changeImmediately, bool forceChange)
{
	bool inputValueUpdated = false;
	int oldGear = GetCurrentActiveGear();
	bool skippingGears = FMath::Abs(oldGear - GearNum) > 1 ? true : false;
	if (oldGear != GearNum)
	{
		inputValueUpdated = true;
	}

	if (!TransmissionSetup.IsCurrentTransmissionModeManual() && !forceChange)
	{
		return false;
	}
	else if (!TransmissionSetup.IsCurrentTransmissionModeManual() && forceChange)
	{
		if (GearNum > 0 && GearNum > TransmissionSetup.ForwardGearRatiosSingular.Num())
		{
			return false;
		}
		else if (GearNum < 0 && (-1 * GearNum) > TransmissionSetup.ReverseGearRatiosSingular.Num())
		{
			return false;
		}
		else
		{
			if (GetCurrentActiveGear() != GearNum + 1 && GetCurrentActiveGear() != GearNum - 1 && GetCurrentActiveGear() != GearNum)
			{
				FVehicleActionErrors actionError;
				actionError.didEngineStall = false;
				actionError.errorReason = EActionErrorReason::GearsSkipped;
				actionError.areValuesFilled = true;
				actionErrorCaused.Broadcast(actionError);
			}
			previousGear = oldGear;
			Super::SetTargetGear(GearNum, changeImmediately);
			gearChanged.Broadcast(oldGear, GearNum, skippingGears);
			UpdateVehicleEngineState();

			if (GearNum < oldGear && GearNum>0)
			{
				isInDownshiftRPMMode = true;
			}

			return true;
		}
	}
	else 
	{
		//Should be able to change gear when engine totally disengaged, within gear change range or in idle.
		if (CanChangeGear())
		{

			if (GearNum > maxGearLock && GearNum>oldGear)
			{
				FVehicleActionErrors actionError;
				actionError.didEngineStall = false;
				actionError.errorReason = EActionErrorReason::AttemptedToSetGearHigherThanMaxGearLock;
				actionError.areValuesFilled = true;
				actionErrorCaused.Broadcast(actionError);

				return false;
			}

			if (GearNum == 0)
			{
				if (GetCurrentActiveGear() != 1 && GetCurrentActiveGear() != -1 && GetCurrentActiveGear() != 0)
				{
					FVehicleActionErrors actionError;
					actionError.didEngineStall = false;
					actionError.errorReason = EActionErrorReason::GearsSkipped;
					actionError.areValuesFilled = true;
					actionErrorCaused.Broadcast(actionError);
				}
				Super::SetTargetGear(0, true);
				previousGear = oldGear;
				gearChanged.Broadcast(oldGear, 0, skippingGears);
				if (GearNum < oldGear && GearNum>0)
				{
					isInDownshiftRPMMode = true;
				}


				UpdateVehicleEngineState();
				return true;
			}

			else if ((GearNum < 0 && GetMovementDirection() == EVehicleMovementDirection::Forward))
			{
				if (vehicleFunctionalities.shouldEngineStallOnCriticalMistakes)
					SetEngineStarterValue(false);
				FVehicleActionErrors actionError;
				actionError.didEngineStall = true && vehicleFunctionalities.shouldEngineStallOnCriticalMistakes;
				actionError.errorReason = EActionErrorReason::ChangeToGearWithMovementInOppositeDirection_Critical;
				actionError.areValuesFilled = true;
				actionErrorCaused.Broadcast(actionError);
			}


			if (vehicleFunctionalities.vehicleHasHighLowGears)
			{
				if (GearNum > 0 && GearNum > TransmissionSetup.ForwardGearRatios.Num())
				{
					return false;
				}
				else if (GearNum < 0 && (-1 * GearNum) > TransmissionSetup.ReverseGearRatios.Num())
				{
					return false;
				}
				else
				{
					if (IsUsingHighGears())
					{
						if (GetCurrentActiveGear() != GearNum + 1 && GetCurrentActiveGear() != GearNum - 1 && GetCurrentActiveGear() != GearNum)
						{
							FVehicleActionErrors actionError;
							actionError.didEngineStall = false;
							actionError.errorReason = EActionErrorReason::GearsSkipped;
							actionError.areValuesFilled = true;
							actionErrorCaused.Broadcast(actionError);
						}
						previousGear = oldGear;
						Super::SetTargetGear(GearNum > 0 ? (GearNum * 2 - 1) : (GearNum * 2 + 1), changeImmediately);
						gearChanged.Broadcast(oldGear, GearNum, skippingGears);
						UpdateVehicleEngineState();
					}
					else
					{
						if (GetCurrentActiveGear() != GearNum + 1 && GetCurrentActiveGear() != GearNum - 1 && GetCurrentActiveGear() != GearNum)
						{
							FVehicleActionErrors actionError;
							actionError.didEngineStall = false;
							actionError.errorReason = EActionErrorReason::GearsSkipped;
							actionError.areValuesFilled = true;
							actionErrorCaused.Broadcast(actionError);
						}
						previousGear = oldGear;
						Super::SetTargetGear(GearNum * 2, changeImmediately);
						gearChanged.Broadcast(oldGear, GearNum, skippingGears);
						UpdateVehicleEngineState();
					}

					if (GearNum < oldGear && GearNum>0)
					{
						isInDownshiftRPMMode = true;
					}

					return true;
				}
			}
			else
			{
				if (GearNum > 0 && GearNum > TransmissionSetup.ForwardGearRatiosSingular.Num())
				{
					return false;
				}
				else if (GearNum < 0 && (-1 * GearNum) > TransmissionSetup.ReverseGearRatiosSingular.Num())
				{
					return false;
				}
				else
				{
					if (GetCurrentActiveGear() != GearNum + 1 && GetCurrentActiveGear() != GearNum - 1 && GetCurrentActiveGear() != GearNum)
					{
						FVehicleActionErrors actionError;
						actionError.didEngineStall = false;
						actionError.errorReason = EActionErrorReason::GearsSkipped;
						actionError.areValuesFilled = true;
						actionErrorCaused.Broadcast(actionError);
					}
					previousGear = oldGear;
					Super::SetTargetGear(GearNum, changeImmediately);
					gearChanged.Broadcast(oldGear, GearNum, skippingGears);
					UpdateVehicleEngineState();

					if (GearNum < oldGear && GearNum>0)
					{
						isInDownshiftRPMMode = true;
					}

					return true;
				}
			}


		}
		else
		{
			FVehicleActionErrors actionError;
			actionError.didEngineStall = false;
			actionError.errorReason = EActionErrorReason::ChangeGearWithoutAppropriateClutch;
			actionError.areValuesFilled = true;
			actionErrorCaused.Broadcast(actionError);
		}
		return false;
	}
}

float UDynamicVehicleMovementComponent::GetVehicleSpeedInKM_PerHour() const
{
	return UKismetMathLibrary::Abs(Chaos::CmSToKmH(GetForwardSpeed()));
}

TArray<float> UDynamicVehicleMovementComponent::GetWheelSpeeds() const
{
	if (derivedPtrForSimulationClass)
	{
		TArray<float> wheelRPMs;
		for (auto Wheel : derivedPtrForSimulationClass->PVehicle->Wheels)
		{
			wheelRPMs.Add(Wheel.GetWheelRPM());
		}
		return wheelRPMs;
	}

	return TArray<float>();
}

float UDynamicVehicleMovementComponent::GetEngineTorque() const
{
	if (derivedPtrForSimulationClass)
	{
		return derivedPtrForSimulationClass->PVehicle->GetEngine().GetEngineTorque();
	}
	return 0.0f;
}

EEngineState UDynamicVehicleMovementComponent::GetCurrentEngineState() const
{
	return currentEngineState;
}

bool UDynamicVehicleMovementComponent::IsVehicleUsingAutomaticTransmission()
{
	return !(TransmissionSetup.IsCurrentTransmissionModeManual());
}

FDynamicInputData UDynamicVehicleMovementComponent::GetCurrentInputData() const
{
	return currentInputs;
}

FTransform UDynamicVehicleMovementComponent::GetCenterOfMass(bool local)
{
	FTransform COMTransform;
	if (local)
	{
		COMTransform = FPhysicsInterface::GetComTransformLocal_AssumesLocked(this->GetBodyInstance()->ActorHandle);
	}
	else
	{
		COMTransform = FPhysicsInterface::GetComTransform_AssumesLocked(this->GetBodyInstance()->ActorHandle);
	}

	if (bEnableCenterOfMassOverride)
	{
		FVector AdjustedCOM = COMTransform.GetTranslation() + CenterOfMassOverride + this->GetBodyInstance()->COMNudge;
		COMTransform.SetTranslation(AdjustedCOM);
	}
	return COMTransform;
}

float UDynamicVehicleMovementComponent::GetNetFuelIntake(float inputGasValue)
{
	float retValue = 0;
	//break assist override
	if (currentInputs.currentBreakAssistValue == true)
	{
		if (GetVehicleSpeedInKM_PerHour() > 5)
			return fuelValueToSustainIdleRPM;
		else
			return 0;
	}
	//normal behaviour
	if (vehicleFunctionalities.vehicleHasManualFuelHandle)
	{
		if (inputGasValue != -1.0f)
		{
			retValue = currentInputs.currentFuelHandleValue > inputGasValue ? (currentInputs.currentFuelHandleValue) : (inputGasValue);
			//retValue = retValue * ((clutchPedalMaxValue - currentInputs.currentClutchPedalValue) / 100);
			return retValue;
		}
		else
		{
			retValue = currentInputs.currentFuelHandleValue > currentInputs.currentGasPedalValue ? (currentInputs.currentFuelHandleValue) : (currentInputs.currentGasPedalValue);
			//retValue = retValue * ((clutchPedalMaxValue - currentInputs.currentClutchPedalValue) / 100);
			return retValue;
		}
	}
	else
	{
		if (inputGasValue != -1.0f)
		{
			return inputGasValue;
		}
		else
		{
			return currentInputs.currentGasPedalValue;

		}
	}

}

FDynamicFunctionalities UDynamicVehicleMovementComponent::GetActiveVehicleFunctionalities() const
{
	return vehicleFunctionalities;
}

FDyamicTransferCaseConfig UDynamicVehicleMovementComponent::GetTransferCaseConfig() const
{
	return transferCaseConfig;
}

void UDynamicVehicleMovementComponent::ToggleLights(bool enable)
{
	if (IsValid(vehicleLights.headLightLeft.fetchedLightComponent))
		vehicleLights.headLightLeft.fetchedLightComponent->SetVisibility(enable,true);
	if (IsValid(vehicleLights.headLightRight.fetchedLightComponent))
		vehicleLights.headLightRight.fetchedLightComponent->SetVisibility(enable,true);
	if (IsValid(vehicleLights.rearLightLeft.fetchedLightComponent))
		vehicleLights.rearLightLeft.fetchedLightComponent->SetVisibility(enable,true);
	if (IsValid(vehicleLights.rearLightRight.fetchedLightComponent))
		vehicleLights.rearLightRight.fetchedLightComponent->SetVisibility(enable,true);
		
	currentInputs.vehicleHeadLightsOn = enable;
	headLightsInputChanged.Broadcast(enable);
	if (!enable)
	{
		headLightsInputReleased.Broadcast();
	}

}

void UDynamicVehicleMovementComponent::ToggleFogLights(bool enable, bool front)
{
	if (front)
	{
		if (IsValid(vehicleLights.fogLightLeft.fetchedLightComponent))
			vehicleLights.fogLightLeft.fetchedLightComponent->SetVisibility(enable, true);
		if (IsValid(vehicleLights.fogLightRight.fetchedLightComponent))
			vehicleLights.fogLightRight.fetchedLightComponent->SetVisibility(enable, true);

		currentInputs.vehicleFrontFogLightsOn = enable;

	}
	else
	{
		if (IsValid(vehicleLights.fogLightLeftRear.fetchedLightComponent))
			vehicleLights.fogLightLeftRear.fetchedLightComponent->SetVisibility(enable, true);
		if (IsValid(vehicleLights.fogLightRightRear.fetchedLightComponent))
			vehicleLights.fogLightRightRear.fetchedLightComponent->SetVisibility(enable, true);

		currentInputs.vehicleRearFogLightsOn = enable;

	}

	fogLightsInputChanged.Broadcast(enable);
	if (!enable)
		fogLightsInputReleased.Broadcast();
	
}

void UDynamicVehicleMovementComponent::ToggleHighbeam(bool enable, bool overRideOnSwitchForLight)
{
	USpotLightComponent* rightLight, *leftLight;
	if (enable && (AreLightsOn(false)|| overRideOnSwitchForLight) && !isOnHighBeamMode)
	{
		if (IsValid(vehicleLights.headLightLeft.fetchedLightComponent))
		{
			leftLight = Cast<USpotLightComponent> (vehicleLights.headLightLeft.fetchedLightComponent->GetChildComponent(0));
			if (IsValid(leftLight))
			{
				leftLight->InnerConeAngle = leftLight->InnerConeAngle + 10;
				leftLight->AttenuationRadius = leftLight->AttenuationRadius + 4000;
				FRotator rotation = leftLight->GetRelativeRotation();
				rotation.Pitch = rotation.Pitch + 10;
				leftLight->SetRelativeRotation(rotation);
				leftLight->Intensity = leftLight->Intensity *3;
				isOnHighBeamMode = true;
			}
		}
		if (IsValid(vehicleLights.headLightRight.fetchedLightComponent))
		{ 
			rightLight = Cast<USpotLightComponent>(vehicleLights.headLightRight.fetchedLightComponent->GetChildComponent(0));
			if (IsValid(rightLight))
			{
				rightLight->InnerConeAngle = rightLight->InnerConeAngle + 10;
				rightLight->AttenuationRadius = rightLight->AttenuationRadius + 4000;
				FRotator rotation = rightLight->GetRelativeRotation();
				rotation.Pitch = rotation.Pitch + 10;
				rightLight->SetRelativeRotation(rotation);
				rightLight->Intensity = rightLight->Intensity * 3;
				isOnHighBeamMode = true;
			}
		}

	}
	else if (!enable && (AreLightsOn(false) || overRideOnSwitchForLight) && isOnHighBeamMode)
	{
		if (IsValid(vehicleLights.headLightLeft.fetchedLightComponent))
		{
			leftLight = Cast<USpotLightComponent>(vehicleLights.headLightLeft.fetchedLightComponent->GetChildComponent(0));
			if (IsValid(leftLight))
			{
				leftLight->InnerConeAngle = leftLight->InnerConeAngle - 10;
				leftLight->AttenuationRadius = leftLight->AttenuationRadius - 4000;
				FRotator rotation = leftLight->GetRelativeRotation();
				rotation.Pitch = rotation.Pitch - 10;
				leftLight->SetRelativeRotation(rotation);
				leftLight->Intensity = leftLight->Intensity / 3;
				isOnHighBeamMode = false;
			}
		}
		if (IsValid(vehicleLights.headLightRight.fetchedLightComponent))
		{
			rightLight = Cast<USpotLightComponent>(vehicleLights.headLightRight.fetchedLightComponent->GetChildComponent(0));
			if (IsValid(rightLight))
			{
				rightLight->InnerConeAngle = rightLight->InnerConeAngle - 10;
				rightLight->AttenuationRadius = rightLight->AttenuationRadius - 4000;
				FRotator rotation = rightLight->GetRelativeRotation();
				rotation.Pitch = rotation.Pitch - 10;
				rightLight->SetRelativeRotation(rotation);
				rightLight->Intensity = rightLight->Intensity / 3;
				isOnHighBeamMode = false;
			}
		}
	}

	currentInputs.vehicleHeadLightsHighBeamOn = enable;

	highBeamLightsInputChanged.Broadcast(isOnHighBeamMode);
	if (!isOnHighBeamMode)
	{
		highBeamLightsInputReleased.Broadcast();
	}
}

void UDynamicVehicleMovementComponent::ToggleIndicators(bool enable, bool left)
{
	if (enable)
	{
		FTimerDelegate TimerDelegate;

		if (left && !leftIndicatorFlicker_TimerHandle.IsValid())
		{
			TimerDelegate.BindUFunction(this, FName("FlickerIndicator"), true);
			GetWorld()->GetTimerManager().SetTimer(leftIndicatorFlicker_TimerHandle, TimerDelegate, .5, true);
			currentInputs.vehicleLeftTurnIndicatorsOn = true;
		}
		else if(!left&&!rightIndicatorFlicker_TimerHandle.IsValid())
		{
			TimerDelegate.BindUFunction(this, FName("FlickerIndicator"), false);
			GetWorld()->GetTimerManager().SetTimer(rightIndicatorFlicker_TimerHandle, TimerDelegate, .5, true);
			currentInputs.vehicleRightTurnIndicatorsOn = true;
		}
	}
	else
	{
		if (left)
		{
			GetWorld()->GetTimerManager().ClearTimer(leftIndicatorFlicker_TimerHandle);
			leftLightFlickerOn = false;
			currentInputs.vehicleLeftTurnIndicatorsOn = false;
			vehicleLights.turnLightLeft.fetchedLightComponent->SetVisibility(false, true);
			vehicleLights.turnLightRight.fetchedLightComponent->SetVisibility(false, true);
			vehicleLights.turnLightRightRear.fetchedLightComponent->SetVisibility(false, true);
			vehicleLights.turnLightLeftRear.fetchedLightComponent->SetVisibility(false, true);

		}
		else
		{
			GetWorld()->GetTimerManager().ClearTimer(rightIndicatorFlicker_TimerHandle);
			rightLightFlickerOn = false;
			currentInputs.vehicleRightTurnIndicatorsOn = false;
			vehicleLights.turnLightLeft.fetchedLightComponent->SetVisibility(false, true);
			vehicleLights.turnLightRight.fetchedLightComponent->SetVisibility(false, true);
			vehicleLights.turnLightRightRear.fetchedLightComponent->SetVisibility(false, true);
			vehicleLights.turnLightLeftRear.fetchedLightComponent->SetVisibility(false, true);
		}

	}
}

void UDynamicVehicleMovementComponent::ToggleIndicatorsManually(bool enable, bool left)
{
	
	if (enable)
	{
		if (IsValid(vehicleSounds.blinkerOnSound))
			PlayIndicatorSound(vehicleSounds.blinkerOnSound, true);
	}
	else
	{
		if (IsValid(vehicleSounds.blinkrOffSound))
			PlayIndicatorSound(vehicleSounds.blinkrOffSound, true);
	}

	if (left)
	{
		if (IsValid(vehicleLights.turnLightLeft.fetchedLightComponent))
		{
			vehicleLights.turnLightLeft.fetchedLightComponent->SetVisibility(enable, true);
		}
		if (IsValid(vehicleLights.turnLightLeftRear.fetchedLightComponent))
		{
			vehicleLights.turnLightLeftRear.fetchedLightComponent->SetVisibility(enable, true);
		}
		currentInputs.vehicleLeftTurnIndicatorsOn = enable;
	}
	else
	{
		if (IsValid(vehicleLights.turnLightRight.fetchedLightComponent))
		{
			vehicleLights.turnLightRight.fetchedLightComponent->SetVisibility(enable, true);
		}
		if (IsValid(vehicleLights.turnLightRightRear.fetchedLightComponent))
		{
			vehicleLights.turnLightRightRear.fetchedLightComponent->SetVisibility(enable, true);
		}
		currentInputs.vehicleRightTurnIndicatorsOn = enable;

	}

}

void UDynamicVehicleMovementComponent::ToggleVehicleHorn(bool enable, bool low)
{
	float volume = low ? 1 : 2;
	if (enable)
	{
		if (IsValid(vehicleSounds.hornSound))
		{
			PlayHorn(vehicleSounds.hornSound, true, volume);
		}
	}
	else
	{
		PlayHorn(nullptr,false);
	}
	currentInputs.vehicleLowHornPressed = enable&&low;
	currentInputs.vehicleHighHornPressed = enable&&!low;
}

void UDynamicVehicleMovementComponent::FlickerIndicator(bool leftLight)
{
	if (!leftLight)
	{
		if (IsValid(vehicleLights.turnLightRight.fetchedLightComponent))
		{
			vehicleLights.turnLightRight.fetchedLightComponent->SetVisibility(!rightLightFlickerOn, true);
		}
		if (IsValid(vehicleLights.turnLightRightRear.fetchedLightComponent))
		{
			vehicleLights.turnLightRightRear.fetchedLightComponent->SetVisibility(!rightLightFlickerOn, true);
		}
		rightLightFlickerOn = !rightLightFlickerOn;
		if (rightLightFlickerOn)
		{
			if (IsValid(vehicleSounds.blinkerOnSound))
				PlayIndicatorSound(vehicleSounds.blinkerOnSound,true);
		}
		else
		{
			if (IsValid(vehicleSounds.blinkrOffSound))
				PlayIndicatorSound(vehicleSounds.blinkrOffSound, true);
		}

	}
	else
	{
		if (IsValid(vehicleLights.turnLightLeft.fetchedLightComponent))
		{
			vehicleLights.turnLightLeft.fetchedLightComponent->SetVisibility(!leftLightFlickerOn, true);
		}
		if (IsValid(vehicleLights.turnLightLeftRear.fetchedLightComponent))
		{
			vehicleLights.turnLightLeftRear.fetchedLightComponent->SetVisibility(!leftLightFlickerOn, true);
		}
		leftLightFlickerOn = !leftLightFlickerOn;

		if (leftLightFlickerOn)
		{
			if (IsValid(vehicleSounds.blinkerOnSound))
				PlayIndicatorSound(vehicleSounds.blinkerOnSound, true);
		}
		else
		{
			if (IsValid(vehicleSounds.blinkrOffSound))
				PlayIndicatorSound(vehicleSounds.blinkrOffSound, true);
		}
	}
}

void UDynamicVehicleMovementComponent::StartFakeRPM()
{
	fakeRPM = PVehicleOutput->EngineRPM;
	useDelayedRPM = true;
	GetWorld()->GetTimerManager().SetTimer(decayedRPMSetter_TimerHandle, this, &UDynamicVehicleMovementComponent::EndFakeRPM, 3.0f, false);

	GetWorld()->GetTimerManager().SetTimer(decayedRPM_TimerHandle, this, &UDynamicVehicleMovementComponent::ModifyFakeRPM, .1f, true);

}

void UDynamicVehicleMovementComponent::EndFakeRPM()
{
	syncRPM = true;
	GetWorld()->GetTimerManager().ClearTimer(decayedRPMSetter_TimerHandle);
	//GetWorld()->GetTimerManager().ClearTimer(decayedRPM_TimerHandle);

}

void UDynamicVehicleMovementComponent::ModifyFakeRPM()
{
	float realRPM = PVehicleOutput->EngineRPM;

	if (!syncRPM)
	{
		if (fakeRPM - realRPM < KINDA_SMALL_NUMBER)
		{
			GetWorld()->GetTimerManager().ClearTimer(decayedRPM_TimerHandle);
			useDelayedRPM = false;
			fakeRPM = realRPM;
			UE_LOG(LogTemp, Warning, TEXT("ModifyFakeRPM-> LINE 3946"));

		}
		else
		fakeRPM = fakeRPM - 1;
	}
	else
	{
		if (fakeRPM - realRPM > KINDA_SMALL_NUMBER)
		{
			fakeRPM = fakeRPM - 50;
		}
		else
		{
			syncRPM = false;
			GetWorld()->GetTimerManager().ClearTimer(decayedRPM_TimerHandle);
			useDelayedRPM = false;
			fakeRPM = realRPM;
		}
	}
	if (fakeRPM < EngineSetup.EngineIdleRPM)
		fakeRPM = EngineSetup.EngineIdleRPM;
}


bool UDynamicVehicleMovementComponent::ApplyGas(float gasPedalValue)
{
	bool inputValueUpdated = false;
	if (!TransmissionSetup.IsCurrentTransmissionModeManual())
	{
		if (currentEngineState == EEngineState::EngineEngaged /*|| currentEngineState == EEngineState::EngineIdle*/)
		{
			float mappedGasValue = UKismetMathLibrary::MapRangeClamped(gasPedalValue, gasPedalMinValue, gasPedalMaxValue, 0, 1);
			SetThrottleInput(mappedGasValue);
			currentInputs.currentGasPedalValue = gasPedalValue;
			if (currentEngineState == EEngineState::EngineIdle && currentInputs.currentGasPedalValue > gasPedalMinValue)
				UpdateVehicleEngineState();
			if (currentEngineState == EEngineState::EngineEngaged && currentInputs.currentGasPedalValue == gasPedalMinValue)
				UpdateVehicleEngineState();
			return true;
		}
		else
		{
			currentInputs.currentGasPedalValue = gasPedalValue;
			return false;
		}
	}
	else
	{
		if ((currentEngineState == EEngineState::EngineEngaged || currentEngineState == EEngineState::EngineGearChangeable))
		{
			float tempGas;

			tempGas = GetNetFuelIntake(gasPedalValue);
			//tempGas = gasPedalValue * ((clutchPedalMaxValue - currentClutchPedalValue) / 100);

			float mappedGasValue = UKismetMathLibrary::MapRangeClamped(tempGas, gasPedalMinValue, gasPedalMaxValue, 0, 1);
			SetThrottleInput(mappedGasValue);

			if (FMath::Abs(currentInputs.currentGasPedalValue - gasPedalValue) > KINDA_SMALL_NUMBER)
				inputValueUpdated = true;

			currentInputs.currentGasPedalValue = gasPedalValue;
		}
		else if (currentEngineState == EEngineState::EngineDisengaged)
		{
			SetThrottleInput(0);

			if (FMath::Abs(currentInputs.currentGasPedalValue - 0) > KINDA_SMALL_NUMBER)
				inputValueUpdated = true;

			currentInputs.currentGasPedalValue = gasPedalValue;
		}
		else
		{
			currentInputs.currentGasPedalValue = gasPedalValue;
		}

		if ((currentInputs.currentGasPedalValue > (gasPedalMaxValue - gasPedalMinValue) * 0.05) && inputValueUpdated)
		{
			gasInputChanged.Broadcast(currentInputs.currentGasPedalValue);
		}
		else if(currentInputs.currentGasPedalValue < (gasPedalMaxValue - gasPedalMinValue) * 0.05)
			gasInputReleased.Broadcast();

		return true;
	}
}

bool UDynamicVehicleMovementComponent::ApplyBrakes(float breakPedalValue)
{
	bool inputValueUpdated = false;

	if (FMath::Abs(currentInputs.currentBreakPedalValue - breakPedalValue) > KINDA_SMALL_NUMBER)
		inputValueUpdated = true;

	currentInputs.currentBreakPedalValue = breakPedalValue;
	float mappedBreakValue = UKismetMathLibrary::MapRangeClamped(breakPedalValue, breakPedalMinValue, breakPedalMaxValue, 0, 1);
	SetBrakeInput(mappedBreakValue);

	if ((currentInputs.currentBreakPedalValue > (breakPedalMaxValue - breakPedalMinValue) * 0.05) && inputValueUpdated)
	{
		brakeInputChanged.Broadcast(currentInputs.currentBreakPedalValue);

		if (IsGasPedalPressed((gasPedalMaxValue - gasPedalMinValue) * .1f))
		{
			FVehicleActionErrors actionError;
			actionError.didEngineStall = true && vehicleFunctionalities.shouldEngineStallOnCriticalMistakes;
			actionError.errorReason = EActionErrorReason::GasAndBrakeInputTogether_Critical;
			actionError.areValuesFilled;
			actionErrorCaused.Broadcast(actionError);
			if(vehicleFunctionalities.shouldEngineStallOnCriticalMistakes)
				SetEngineStarterValue(false);

		}

	}
	else if(currentInputs.currentBreakPedalValue < (breakPedalMaxValue - breakPedalMinValue) * 0.05)
		brakeInputReleased.Broadcast();

	if (vehicleLights.useVehicleLights)
	{
		if (IsValid(vehicleLights.brakeLightLeft.fetchedLightComponent) && IsValid(vehicleLights.brakeLightRight.fetchedLightComponent))
		{
			if (mappedBreakValue > 0)
			{
				vehicleLights.brakeLightLeft.fetchedLightComponent->SetVisibility(true);
				vehicleLights.brakeLightRight.fetchedLightComponent->SetVisibility(true);
			}
			else if(IsBreakActiveInAnyForm(mappedBreakValue>0)!=true)
			{
				vehicleLights.brakeLightLeft.fetchedLightComponent->SetVisibility(false);
				vehicleLights.brakeLightRight.fetchedLightComponent->SetVisibility(false);
			}
		}
	}


	return true;
}

bool UDynamicVehicleMovementComponent::SteerVehicle(float steeringWheelValue)
{
	currentInputs.currentSteeringWheelValue = steeringWheelValue;
	float mappedSteerValue = UKismetMathLibrary::MapRangeClamped(steeringWheelValue, steerWheelMinValue, steerWheelMaxValue, -1, 1);
	SetSteeringInput(mappedSteerValue);
	return true;
}

bool UDynamicVehicleMovementComponent::ApplyClutch(float clutchPedalValue)
{
	bool inputValueUpdated = false;
	clutchPedalValue = UKismetMathLibrary::MapRangeClamped(clutchPedalValue, clutchPedalMinValue, clutchPedalMaxValue, 0, 100);
	currentTransmissionRatio = UKismetMathLibrary::MapRangeClamped(clutchPedalValue, clutchPedalMinValue, clutchPedalMaxValue, 1, 0);

	if (FMath::Abs(currentInputs.currentClutchPedalValue - clutchPedalValue) > KINDA_SMALL_NUMBER)
		inputValueUpdated = true;

	if (!TransmissionSetup.IsCurrentTransmissionModeManual())
	{ 
		return false;
	}
	else
	{
		if (currentInputs.currentClutchPedalValue < clutchThresholdValue && clutchPedalValue >= clutchThresholdValue)
		{
			currentInputs.currentClutchPedalValue = clutchPedalValue;
			UpdateVehicleEngineState();

			//currentGasPedalValue < 1 (for range 0-100)
			if (GetNetFuelIntake() < (gasPedalMinValue + (gasPedalMaxValue - gasPedalMinValue) * .01f))
			{
				float throttleWhenLeavingClutch = (clutchPedalMaxValue - currentInputs.currentClutchPedalValue) * .2;
				SetThrottleInput(UKismetMathLibrary::MapRangeClamped(throttleWhenLeavingClutch, gasPedalMinValue, gasPedalMaxValue, 0, 1));
			}

		}
		else if (currentInputs.currentClutchPedalValue >= clutchThresholdValue && clutchPedalValue < clutchThresholdValue)
		{
			currentInputs.currentClutchPedalValue = clutchPedalValue;
			if (canEngineJumpStart)
			{
				SetEngineStarterValue(true);
			}
			else
			{
				UpdateVehicleEngineState();
			}
		}
		else if (currentInputs.currentClutchPedalValue >= clutchDisengagementValue && clutchPedalValue < clutchDisengagementValue)
		{
			currentInputs.currentClutchPedalValue = clutchPedalValue;
			UpdateVehicleEngineState();
		}
		else if (currentInputs.currentClutchPedalValue < clutchDisengagementValue && clutchPedalValue >= clutchDisengagementValue)
		{
			currentInputs.currentClutchPedalValue = clutchPedalValue;
			UpdateVehicleEngineState();
		}
		else if ( clutchPedalValue < clutchThresholdValue && clutchPedalValue > (clutchPedalMinValue + (clutchPedalMaxValue - clutchPedalMinValue)*.1f) 
					&& currentEngineState == EEngineState::EngineEngaged && GetNetFuelIntake() < (gasPedalMinValue + (gasPedalMaxValue-gasPedalMinValue)*.05f) )
		{
			float currentVehicleSpeed = GetVehicleSpeedInKM_PerHour();
			float currentGear = GetCurrentActiveGearRatio();
			float currentGearMinimumSpeed = 0;
			if (currentGear > 0)
			{
				currentGearMinimumSpeed = TransmissionSetup.GetMinimumSpeedForGear(currentGear , true);
			}
			else if (currentGear < 0)
			{
				currentGearMinimumSpeed = TransmissionSetup.GetMinimumSpeedForGear(currentGear * -1 , false);
			}
			float throttleWhenLeavingClutch = ((clutchPedalMaxValue - clutchPedalValue) * .5);
			if (currentVehicleSpeed > currentGearMinimumSpeed)
			{
				throttleWhenLeavingClutch = throttleWhenLeavingClutch * UKismetMathLibrary::Exp(-0.06 * (currentVehicleSpeed * currentGearMinimumSpeed));
			}
			SetThrottleInput(UKismetMathLibrary::MapRangeClamped(throttleWhenLeavingClutch, gasPedalMinValue, gasPedalMaxValue, 0, 1));
			UE_LOG(LogTemp, Log, TEXT("Throttle input by clutch : %s"), *FString::SanitizeFloat(throttleWhenLeavingClutch));

			currentInputs.currentClutchPedalValue = clutchPedalValue;

		}
		else
			currentInputs.currentClutchPedalValue = clutchPedalValue;

		if ((currentInputs.currentClutchPedalValue > (clutchPedalMaxValue - clutchPedalMinValue) * 0.05) && inputValueUpdated)
		{
			clutchInputChanged.Broadcast(currentInputs.currentClutchPedalValue);
		}
		else if (currentInputs.currentClutchPedalValue < (clutchPedalMaxValue - clutchPedalMinValue) * 0.05)
			clutchInputReleased.Broadcast();

		return true;
	}
}

bool UDynamicVehicleMovementComponent::ChangeHandBrakeState(bool handBreakActive)
{
	bool inputValueUpdated = false;

	if (currentInputs.currentHandbreakValue != handBreakActive)
		inputValueUpdated = true;


	currentInputs.currentHandbreakValue = handBreakActive;
	SetHandbrakeInput(handBreakActive);

	if (inputValueUpdated)
	{
		handbrakeInputChanged.Broadcast(handBreakActive);
		if (!handBreakActive)
			handbrakeInputReleased.Broadcast();
	}

	if (vehicleLights.useVehicleLights)
	{
		if (IsValid(vehicleLights.brakeLightLeft.fetchedLightComponent) && IsValid(vehicleLights.brakeLightRight.fetchedLightComponent))
		{
			if (handBreakActive)
			{
				vehicleLights.brakeLightLeft.fetchedLightComponent->SetVisibility(true);
				vehicleLights.brakeLightRight.fetchedLightComponent->SetVisibility(true);
			}
			else if (IsBreakActiveInAnyForm()!=true)
			{
				vehicleLights.brakeLightLeft.fetchedLightComponent->SetVisibility(false);
				vehicleLights.brakeLightRight.fetchedLightComponent->SetVisibility(false);
			}
		}
	}

	return true;
}

bool UDynamicVehicleMovementComponent::IsBreakActiveInAnyForm(bool overrideValue) const
{
	//checks for all forms of breaking.
	return ((currentInputs.currentHandbreakValue) || (currentInputs.currentBreakPedalValue > breakPedalMinValue) || (currentInputs.currentBreakAssistValue) || overrideValue);
}

bool UDynamicVehicleMovementComponent::IsReversing()
{
	return GetCurrentActiveGear()<0;
}

float UDynamicVehicleMovementComponent::GetTraveledDistance()
{
	return TotalDistanceTraveled;
}

bool UDynamicVehicleMovementComponent::IsVehicleOnSlope(float& outSlopeAngle, float AngleThreshold)
{
	outSlopeAngle = 0;
	// Get the vehicle's mesh component
	USkeletalMeshComponent* Mesh = Cast<USkeletalMeshComponent>(GetMesh());
	if (!Mesh)
	{
		return false; // Mesh not found, return false
	}

	// Get the bounding box of the mesh to determine the vehicle size
	FVector MeshOrigin, MeshExtent;
	FBoxSphereBounds localBounds = Mesh->Bounds;
	MeshExtent = localBounds.BoxExtent;
	MeshOrigin = localBounds.Origin;

	// Calculate trace offsets based on the mesh size
	const FVector VehicleLocation = GetOwner()->GetActorLocation();
	const FVector VehicleUpVector = GetOwner()->GetActorUpVector();
	const float FrontOffset = MeshExtent.X;
	const float BackOffset = MeshExtent.X;
	const float SideOffset = MeshExtent.Y;
	const float TraceDistance = MeshExtent.Z * 4.0f; // Adjust trace distance to cover the height of the vehicle

	// Points around the vehicle to trace from
	TArray<FVector> TracePoints = {
		VehicleLocation + FVector(FrontOffset, 0, 0),
		VehicleLocation + FVector(-BackOffset, 0, 0),
		VehicleLocation + FVector(0, SideOffset, 0),
		VehicleLocation + FVector(0, -SideOffset, 0)
	};

	TArray<FHitResult> HitResults;
	for (const FVector& Point : TracePoints)
	{
		FHitResult Hit;
		FVector Start = Point;
		FVector End = Point - FVector(0, 0, TraceDistance);

		// Perform line trace
		if (GetWorld()->LineTraceSingleByChannel(Hit, Start, End, ECC_Visibility))
		{
			HitResults.Add(Hit);
		}
	}

	// If all line traces hit the ground, calculate the average normal
	if (HitResults.Num() == TracePoints.Num())
	{
		FVector NormalSum = FVector::ZeroVector;
		for (const FHitResult& Hit : HitResults)
		{
			NormalSum += Hit.Normal;
		}

		FVector AverageNormal = NormalSum / HitResults.Num();

		// Calculate the slope angle in degrees
		float SlopeAngle = FMath::RadiansToDegrees(FMath::Acos(FVector::DotProduct(AverageNormal, VehicleUpVector)));

		// Check if the slope angle exceeds the threshold
		outSlopeAngle = SlopeAngle;
		return SlopeAngle > AngleThreshold;
	}

	// If we don't hit the ground with all traces, assume no slope
	return false;
}

ESlideDirection UDynamicVehicleMovementComponent::GetSlideDirection()
{
	float returnedSlopeAngle;
	if (!IsVehicleOnSlope(returnedSlopeAngle))
	{
		// Not on a slope, so not sliding
		return ESlideDirection::NotSliding;
	}

	// Get the actor's velocity
	FVector currVelocity = GetOwner()->GetVelocity();

	// Check if the velocity indicates movement
	if (currVelocity.IsNearlyZero())
	{
		// Not moving, so not sliding
		return ESlideDirection::NotSliding;
	}

	// Get the actor's forward vector
	FVector ForwardVector = GetOwner()->GetActorForwardVector();

	// Normalize the velocity to get the direction of movement
	FVector SlideDirection = currVelocity.GetSafeNormal();

	// Calculate the dot product between the forward vector and the slide direction
	float DotProduct = FVector::DotProduct(ForwardVector, SlideDirection);

	// Determine the sliding direction based on the dot product
	if (DotProduct > 0.0f)
	{
		// Sliding forward
		return ESlideDirection::SlidingForward;
	}
	else if (DotProduct < 0.0f)
	{
		// Sliding backward
		return ESlideDirection::SlidingBackward;
	}

	// Fallback case (this should rarely occur due to IsNearlyZero check)
	return ESlideDirection::NotSliding;
}

EVehicleMovementDirection UDynamicVehicleMovementComponent::GetMovementDirection()
{
	float ForwardSpeed = Chaos::CmSToKmH(GetForwardSpeed());

	if (ForwardSpeed > 1) // Forward movement
	{
		return EVehicleMovementDirection::Forward;
	}
	else if (ForwardSpeed < -1) // Backward movement
	{
		return EVehicleMovementDirection::Backward;
	}

	// No significant movement
	return EVehicleMovementDirection::None;
}

bool UDynamicVehicleMovementComponent::AreLightsOn(bool checkFogLights)
{
	bool retValue = false;
	if (checkFogLights)
	{
		if (IsValid(vehicleLights.fogLightLeft.fetchedLightComponent))
			retValue = vehicleLights.fogLightLeft.fetchedLightComponent->IsVisible();
		if (IsValid(vehicleLights.fogLightLeftRear.fetchedLightComponent))
			retValue = vehicleLights.fogLightLeftRear.fetchedLightComponent->IsVisible();
		if (IsValid(vehicleLights.fogLightRight.fetchedLightComponent))
			retValue = vehicleLights.fogLightRight.fetchedLightComponent->IsVisible();
		if (IsValid(vehicleLights.fogLightRightRear.fetchedLightComponent))
			retValue = vehicleLights.fogLightRightRear.fetchedLightComponent->IsVisible();
	}
	else
	{
		if (IsValid(vehicleLights.headLightLeft.fetchedLightComponent))
			retValue = vehicleLights.headLightLeft.fetchedLightComponent->IsVisible();
		if (IsValid(vehicleLights.headLightRight.fetchedLightComponent))
			retValue = vehicleLights.headLightRight.fetchedLightComponent->IsVisible();
		if (IsValid(vehicleLights.rearLightLeft.fetchedLightComponent))
			retValue = vehicleLights.rearLightLeft.fetchedLightComponent->IsVisible();
		if (IsValid(vehicleLights.rearLightRight.fetchedLightComponent))
			retValue = vehicleLights.rearLightRight.fetchedLightComponent->IsVisible();
	}
	return retValue;
}

bool UDynamicVehicleMovementComponent::IsGasPedalPressed(float aboveValue)
{
	if (currentInputs.currentGasPedalValue > gasPedalMinValue)
	{
		if (currentInputs.currentGasPedalValue > aboveValue)
		{
			return true;
		}
		else
			return false;
	}
	else return false;
}

ETransmissionType UDynamicVehicleMovementComponent::GetVehicleTransmissionType()
{
	return TransmissionSetup.vehicleTransmissionType;
}

bool UDynamicVehicleMovementComponent::IsVehicleInManualMode()
{
	return TransmissionSetup.IsCurrentTransmissionModeManual();
}

int UDynamicVehicleMovementComponent::GetCurrentMaxGear()
{
	return maxGearLock;
}

bool UDynamicVehicleMovementComponent::IsTurboModeActive()
{
	return TurboModeActive;
}

float UDynamicVehicleMovementComponent::GetTurboModeDuration()
{
	return vehicleFunctionalities.vehicleHasTurboMode ? turboModeDuration : 0;
}

float UDynamicVehicleMovementComponent::GetTurboModeIncreaseFactor()
{
	return vehicleFunctionalities.vehicleHasTurboMode ? turboModeFactor : 0;
}

bool UDynamicVehicleMovementComponent::IsABS_InUse()
{
	bool retVal = false;

	if (VehicleSimulationPT && VehicleSimulationPT->PVehicle && VehicleSimulationPT->PVehicle->Wheels.Num() > 0)
	{
		for (int i = 0; i < VehicleSimulationPT->PVehicle->Wheels.Num(); i++)
		{
			retVal = retVal || VehicleSimulationPT->PVehicle->Wheels[i].bABSActivated;
		}
	}
	return retVal;
}

void UDynamicVehicleMovementComponent::SetTurboModeData(float duration, float increaseFactor)
{
	if (vehicleFunctionalities.vehicleHasTurboMode)
	{
		turboModeDuration = FMath::Max(duration, 0);
		turboModeFactor = FMath::Max(increaseFactor, 0);
	}
}

void UDynamicVehicleMovementComponent::ActivateTurboMode()
{
	if (vehicleFunctionalities.vehicleHasTurboMode)
	{
		if (derivedPtrForSimulationClass)
		{
			derivedPtrForSimulationClass->dataImpactingSimulation.ModifyDriveTorqueFactor(turboModeFactor);
			TurboModeActive = true;
			GetWorld()->GetTimerManager().SetTimer(turboMode_TimerHandle, this, &UDynamicVehicleMovementComponent::DeactivateTurboMode, turboModeDuration);
		}
	}
}

void UDynamicVehicleMovementComponent::DeactivateTurboMode()
{
	if (derivedPtrForSimulationClass)
	{
		derivedPtrForSimulationClass->dataImpactingSimulation.ModifyDriveTorqueFactor(1);
		TurboModeActive = false;
		GetWorld()->GetTimerManager().ClearTimer(turboMode_TimerHandle);
	}
}

void UDynamicVehicleMovementComponent::ToggleTrackingTravelDistance(bool enable)
{
	if (enable)
	{
		if (shouldTrackTravelDistance)
		{
			//Do Nothing
		}
		else
		{
			TotalDistanceTraveled = 0;
			shouldTrackTravelDistance = true;
		}
	}
	else
	{
		shouldTrackTravelDistance = false;
	}
}

bool UDynamicVehicleMovementComponent::SetEngineStarterValue(bool starterValue)
{
	if (currentEngineStartedValue == starterValue)
		return false;
	else if(starterValue==true)
	{
		if (GetCurrentActiveGear() != 0)
		{
			if (currentInputs.currentClutchPedalValue >= clutchThresholdValue && vehicleFunctionalities.vehicleCanStartWithoutNeutralGear)
			{
				currentEngineStartedValue = true;
				canEngineJumpStart = false;
				vehicleStarted.Broadcast();
			}
			else if (canEngineJumpStart)
			{
				//temporary jumpstart condition
				currentEngineStartedValue =  true;
				canEngineJumpStart = false;
				vehicleStarted.Broadcast();
				UE_LOG(LogTemp, Log, TEXT("Starter allowed jump start"));
			}
			else if(vehicleFunctionalities.vehicleCanStartWithoutNeutralGear)
			{
				//Can not start truck directly in gear without clutch pressed
				FVehicleActionErrors actionError;
				actionError.didEngineStall = true;
				actionError.errorReason = EActionErrorReason::AttemptToStartInGearWithoutClutch;
				actionError.areValuesFilled = true;
				actionErrorCaused.Broadcast(actionError);
				
				return false;
			}
		}
		else
		{
			currentEngineStartedValue = true;
			canEngineJumpStart = false;
			vehicleStarted.Broadcast();
		}
	}
	else
	{
		currentEngineStartedValue = false;
		vehicleStopped.Broadcast();
	}
	UpdateVehicleEngineState();
	return true;
}

void UDynamicVehicleMovementComponent::BeginPlay()
{
	Super::BeginPlay();
	vehicleLights.ExtractAll(GetOwner());
	vehicleNonFunctionalAspects.leftWiper.ExtractMeshComponent(GetOwner());
	vehicleNonFunctionalAspects.rightwiper.ExtractMeshComponent(GetOwner());

	playerController = UGameplayStatics::GetPlayerController(GetWorld(),0);
	if (TransmissionSetup.vehicleTransmissionType != ETransmissionType::Manual)
		currentTransmissionRatio = 1;
	TransmissionSetup.SetTransferCaseModifier(transferCaseConfig.GetTransferCaseRatio());
	currentTransferCaseRatio = transferCaseConfig.GetTransferCaseRatio();

	if (TransmissionSetup.vehicleTransmissionType == ETransmissionType::Manual && TransmissionSetup.bUseHighLowRatios)
		maxGearLock = TransmissionSetup.ForwardGearRatios.Num();
	else
		maxGearLock = TransmissionSetup.ForwardGearRatiosSingular.Num(); 

	if (derivedPtrForSimulationClass)
	{
		derivedPtrForSimulationClass->dataImpactingSimulation.SetMaxGearLock(maxGearLock);
		derivedPtrForSimulationClass->SetDifferentialSystem(GetCurrentDifferentialSystem());
	}

	SetDifferentialMode(DifferentialSetup.defaultDifferentialMode);
	

	for (int i =0; i<Wheels.Num(); i++)
	{
		SetABSEnabled(i, vehicleFunctionalities.vehicleHasABS);
		
	}

}

void UDynamicVehicleMovementComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	bool overRideSlopeBreak = false;
	float currentVehicleSpeed = GetVehicleSpeedInKM_PerHour();
	
	//gear based Breaking ***************************
	float currentGear = GetCurrentActiveGear();
	float maxSpeedForGear = BIG_NUMBER;
	if (currentGear > 0)
	{
		maxSpeedForGear = TransmissionSetup.GetMaximumSpeedForGear(currentGear , true);
	}
	else if (currentGear < 0)
	{
		maxSpeedForGear = TransmissionSetup.GetMaximumSpeedForGear(currentGear * -1 , false);
	}
	if (currentVehicleSpeed > maxSpeedForGear && maxSpeedForGear!=-1)
	{
		SetHandbrakeInput(true);
		overRideSlopeBreak = true;
		
		if (!doOnceForEnableSlideConsoleCommand)
		{
			FString Command = FString::Printf(TEXT("p.Chaos.Suspension.SlopeThreshold 0"));

			playerController->ConsoleCommand(Command);
			doOnceForEnableSlideConsoleCommand = true;
		}
	}
	else
	{
		if(!currentInputs.currentHandbreakValue)
			SetHandbrakeInput(false);
		overRideSlopeBreak = false;

	}
	//gear based Breaking ends here ***************************

	if (!TransmissionSetup.IsCurrentTransmissionModeManual() && currentEngineState != EEngineState::EngineOff)
	{
		if (currentGearCached != currentGear)
		{
			previousGear = currentGearCached;
			currentGearCached = currentGear;

			gearChanged.Broadcast(previousGear, currentGearCached, FMath::Abs(previousGear- currentGearCached)>1);

			if (currentGearCached < previousGear)
			{
				isInDownshiftRPMMode = true;
			}

		}
	}


	//setting downshift status ********************************
	//as a workaround for proper engine breaking effect on RPM even with RPM limited by fuel
	if (derivedPtrForSimulationClass && isInDownshiftRPMMode)
	{
		
		float tempVehicleRPM = derivedPtrForSimulationClass->GetCalculatedRPM(GetWheelSpeeds()[0]);
		float EngineRPM = PVehicleOutput->EngineRPM;
		if (EngineRPM <= tempVehicleRPM)
		{
			isInDownshiftRPMMode = false;
		}
	}
	//setting downshift status ends here ***********************
	

	//set Accelerate/Decelerate Bool ***************************
	if (previousVehicleSpeed - currentVehicleSpeed >= KINDA_SMALL_NUMBER)
	{
		isVehicleAccelerating = false;
		previousVehicleSpeed = currentVehicleSpeed;
	}
	else if ( currentVehicleSpeed - previousVehicleSpeed >= KINDA_SMALL_NUMBER)
	{
		isVehicleAccelerating = true;
		previousVehicleSpeed = currentVehicleSpeed;
	}
	//set Accelerate/Decelerate Bool ends here *****************

	//Action Error check when Engine Engaged + Engine Update *******************
	if (currentEngineState == EEngineState::EngineEngaged)
	{
		UpdateVehicleEngineState();

		if (TransmissionSetup.GetTransferCaseModifer() == 0)
		{
			if (doOnceForLockedAxelError)
			{
				doOnceForLockedAxelError = false;
				FVehicleActionErrors actionError;
				actionError.didEngineStall = false;
				actionError.errorReason = EActionErrorReason::EngineEngagedOnNeutralTransferCase;
				actionError.areValuesFilled = true;
				actionErrorCaused.Broadcast(actionError);
			}
		}
		if (currentInputs.currentHandbreakValue == true)
		{
			if (doOnceForParkingBreakError)
			{
				doOnceForParkingBreakError = false;
				FVehicleActionErrors actionError;
				actionError.didEngineStall = false;
				actionError.errorReason = EActionErrorReason::DriveWithHandBreakOn;
				actionError.areValuesFilled = true;
				actionErrorCaused.Broadcast(actionError);
			}
		}

	}
	else if(TransmissionSetup.vehicleTransmissionType != ETransmissionType::Manual && GetCurrentActiveGear()!=0 && currentEngineStartedValue)
	{
		UpdateVehicleEngineState();
	}
	//Action Error check when Engine Engaged + Engine Update ends here **********

	//Precautionary throttle to 0, when engine state off, since engine is never truly off in chaos vehicles *******************
	if (currentEngineState == EEngineState::EngineOff || currentEngineState == EEngineState::EngineIdle)
	{
		SetThrottleInput(0);
	}
	//ends here *******************

	//Transferring data to simulation class here *******************
	if (currentEngineState != EEngineState::EngineOff && currentEngineStartedValue!= false)
	{
		if (derivedPtrForSimulationClass)
		{
			derivedPtrForSimulationClass->dataImpactingSimulation.FillData(GetNetFuelIntake(),isInDownshiftRPMMode, GetVehicleSpeedInKM_PerHour(), currentTransmissionRatio, currentInputs.currentGasPedalValue > gasPedalMinValue, currentInputs.currentBreakAssistValue);
		}
	}
	else
	{
		if (derivedPtrForSimulationClass)
		{
			derivedPtrForSimulationClass->dataImpactingSimulation.isEngineStarted = false;
		}
	}
	//ends here *******************

	//Engine Jump Start Condition Setup *******************
	if (currentEngineStartedValue == false && (currentEngineState == EEngineState::EngineGearChangeable || currentEngineState == EEngineState::EngineDisengaged))
	{
		if (derivedPtrForSimulationClass)
		{
			float engineJumpstartRPM = derivedPtrForSimulationClass->GetRelativeEngineRPM_FromSpeed(currentVehicleSpeed);
			if (engineJumpstartRPM > EngineSetup.EngineIdleRPM)
			{
				canEngineJumpStart = true;
			}
			else
			{
				canEngineJumpStart = false;
			}
		}
	}
	//ends here *******************

	//Distance Tracking *******************
	if (shouldTrackTravelDistance)
	{
		FVector currentPosition = GetActorLocation();
		float DistanceTraveledThisFrame = FVector::Dist(currentPosition, previousPosition);
		TotalDistanceTraveled += DistanceTraveledThisFrame;
		previousPosition = currentPosition;
	}
	//ends here *******************
	
	//Vehicle Slide on Slope *******************
	if (vehicleFunctionalities.vehicleShouldSlideOnSlope && !overRideSlopeBreak)
	{
		float slopeAngle = 0;
		if (IsVehicleOnSlope(slopeAngle))
		{
			 currentGear = GetCurrentActiveGear();
			 maxSpeedForGear = BIG_NUMBER;
			if(currentGear>0)
				 maxSpeedForGear = TransmissionSetup.GetMaximumSpeedForGear(currentGear, true);
			else if (currentGear<0)
				maxSpeedForGear = TransmissionSetup.GetMaximumSpeedForGear(currentGear*-1 , false);

			float currSpeed = GetVehicleSpeedInKM_PerHour();
			float currRPM = GetEngineRotationSpeed();
			if (currentInputs.currentBreakPedalValue > breakPedalMinValue || (currRPM - EngineSetup.EngineIdleRPM > 1 && ((IsUsingHighGears()==false && slopeAngle < maxAngleAllowedLowGears) || (IsUsingHighGears() && slopeAngle < maxAngleAllowed))) || (currSpeed > maxSpeedForGear))
			{
				if (doOnceForDisableSlideConsoleCommand)
				{
					if (IsValid(playerController))
					{
						float cosValue = FMath::Cos(FMath::DegreesToRadians(IsUsingHighGears() ? maxAngleAllowed : maxAngleAllowedLowGears));
						FString Command = FString::Printf(TEXT("p.Chaos.Suspension.SlopeThreshold %s"), *FString::SanitizeFloat(cosValue));

						playerController->ConsoleCommand(Command);

						UE_LOG(LogTemp, Log, TEXT("%s"), *Command);
						doOnceForEnableSlideConsoleCommand = true;
						doOnceForDisableSlideConsoleCommand = false;


						if (currSpeed > maxSpeedForGear && maxSpeedForGear!=-1)
						{
							SetHandbrakeInput(true);
							doOnceForEnableSlideConsoleCommand = true;
							doOnceForDisableSlideConsoleCommand = true;
						}
						else if (!currentInputs.currentHandbreakValue)
						{
							SetHandbrakeInput(false);
						}

					}
				}
			}
			else if((currSpeed < maxSpeedForGear))
			{
				if (doOnceForEnableSlideConsoleCommand)
				{
					if (IsValid(playerController))
					{
						FString Command = FString::Printf(TEXT("p.Chaos.Suspension.SlopeThreshold 1"));
						playerController->ConsoleCommand(Command);
						UE_LOG(LogTemp, Log, TEXT("%s"), *Command);
						doOnceForEnableSlideConsoleCommand = false;
						doOnceForDisableSlideConsoleCommand = true; //allow disable because it is enabled now

						if (!currentInputs.currentHandbreakValue)
							SetHandbrakeInput(false);
					}
				}
			}

			
		}
		else
		{
			doOnceForEnableSlideConsoleCommand = true;
			doOnceForDisableSlideConsoleCommand = true;
			if (!currentInputs.currentHandbreakValue)
			{
				SetHandbrakeInput(false);
			}
		}
	}
	//ends here *******************

	//Fix for gear not changing in some diff modes for auto
	if (!TransmissionSetup.IsCurrentTransmissionModeManual() && IsEngineStarted() && (currentDifferentialMode == EDynamicDifferentialModes::AllDifferentials || currentDifferentialMode == EDynamicDifferentialModes::RearDifferentialLock))
	{
		float newGear = 0;
		float currRPM = GetEngineRotationSpeed();
		int currGear = GetCurrentActiveGear();
		if (currRPM > TransmissionSetup.ChangeUpRPM && currGear > 0)
		{
			newGear = GetCurrentActiveGear() + 1;
			SetNewGear(newGear, true, true);
		}
		if (currRPM < TransmissionSetup.ChangeDownRPM && currGear > 0)
		{
			newGear = FMath::Clamp(GetCurrentActiveGear() - 1, 1, TransmissionSetup.ForwardGearRatios.Num() );
			SetNewGear(newGear, true, true);
		}
	}
}

void UDynamicVehicleMovementComponent::UpdateVehicleEngineState()
{
	if (!(TransmissionSetup.vehicleTransmissionType == ETransmissionType::Manual))
	{
		EEngineState valueToSet = EEngineState::EngineOff; //we start with off, process down till all conditions are checked

		//Most importantly checking if engine starter has been turned off
		if (currentEngineStartedValue == false)
		{
			valueToSet = EEngineState::EngineOff;
			//Engine off. no other processing needed.
		}
		else
		{
			int gear = GetCurrentActiveGear();
			if (gear >0 || gear < 0)
			{
				valueToSet = EEngineState::EngineEngaged;
			}
			else if(gear==0)
			{
				valueToSet = EEngineState::EngineIdle;
			}
			else if(currentEngineState!= EEngineState::EngineEngaged)
			{
				valueToSet = EEngineState::EngineIdle;
			}
			else
			{
				valueToSet = EEngineState::EngineEngaged;
			}
		}
		currentEngineState = valueToSet;

	}
	else
	{

		/*
		Engine State changes in following ways:
		1. Vehicle was off and has now been started. Off -> Idle / Off -> Engaged
		2. Vehicle was in idle and Clutch has been pressed above threshold. Idle -> GearChangeable / Idle -> DisEngaged
		3. Clutch gets engaged but above threshold. Disengaged -> GearChangeable
		4. Clutch goes below threshold. GearChangeable -> Engaged / GearChangeable -> Idle
		6. Vehicle is stopped and ignition is turned off. Any -> Off
		7. Vehicle in Gear, but no gas or clutch input and speed goes below minimum speed for gear. Engaged -> Off
		*/
		FVehicleActionErrors actionError;
		EEngineState valueToSet = EEngineState::EngineOff; //we start with off, process down till all conditions are checked

		//Most importantly checking if engine starter has been turned off
		if (currentEngineStartedValue == false)
		{
			valueToSet = EEngineState::EngineOff;

			if (currentInputs.currentClutchPedalValue >= clutchThresholdValue)
			{
				if (currentInputs.currentClutchPedalValue >= clutchDisengagementValue)
				{
					valueToSet = EEngineState::EngineDisengaged;
				}
				else
				{
					valueToSet = EEngineState::EngineGearChangeable;
				}
			}
			//Engine off. No other processing needed.
		}
		else
		{
			float currentSpeed = GetVehicleSpeedInKM_PerHour();
			//The following processing may look repetetive but it is done in such a way to allow future processing to be added in any case.
			if (currentEngineState == EEngineState::EngineOff)
			{
				if ((GetNetFuelIntake() >= fuelValueToSustainIdleRPM) || (GetNetFuelIntake() < fuelValueToSustainIdleRPM && currentInputs.currentClutchPedalValue> (clutchPedalMaxValue - clutchPedalMinValue)*0.1f ))
				{
					valueToSet = EEngineState::EngineIdle;
					if (currentInputs.currentClutchPedalValue >= clutchThresholdValue)
					{
						if (currentInputs.currentClutchPedalValue >= clutchDisengagementValue)
						{
							valueToSet = EEngineState::EngineDisengaged;
						}
						else
						{
							valueToSet = EEngineState::EngineGearChangeable;
						}
					}
				}
				else
				{
					valueToSet = EEngineState::EngineOff;
				}
			}
			if (currentEngineState == EEngineState::EngineIdle)
			{
				if ((GetNetFuelIntake() >= fuelValueToSustainIdleRPM) || (GetNetFuelIntake() < fuelValueToSustainIdleRPM && currentInputs.currentClutchPedalValue> (clutchPedalMaxValue - clutchPedalMinValue)*0.1f ))
				{
					if (currentInputs.currentClutchPedalValue >= clutchThresholdValue)
					{
						if (currentInputs.currentClutchPedalValue >= clutchDisengagementValue)
						{
							valueToSet = EEngineState::EngineDisengaged;
						}
						else
						{
							valueToSet = EEngineState::EngineGearChangeable;
						}
					}
					else if (currentInputs.currentClutchPedalValue < clutchThresholdValue)
					{
						if (GetCurrentActiveGear() != 0 && IsGasPedalPressed((gasPedalMaxValue-gasPedalMinValue)*.1f))
						{
							valueToSet = EEngineState::EngineEngaged;
						}
						else
						{
							valueToSet = EEngineState::EngineIdle;
						}
					}
				}
				else
				{
					actionError.didEngineStall = true;
					actionError.errorReason = EActionErrorReason::NoGasOrClutch;
					actionError.areValuesFilled = true;
					valueToSet = EEngineState::EngineOff;

				}
			}
			if (currentEngineState == EEngineState::EngineEngaged)
			{
				if ((GetNetFuelIntake() >= fuelValueToSustainIdleRPM) || (GetNetFuelIntake() < fuelValueToSustainIdleRPM && currentInputs.currentClutchPedalValue> (clutchPedalMaxValue - clutchPedalMinValue)*0.1f ))
				{
					if (currentInputs.currentClutchPedalValue >= clutchThresholdValue)
					{
						if (currentInputs.currentClutchPedalValue >= clutchDisengagementValue)
						{
							valueToSet = EEngineState::EngineDisengaged;
						}
						else
						{
							valueToSet = EEngineState::EngineGearChangeable;
						}
					}
					else if (currentInputs.currentClutchPedalValue < clutchThresholdValue)
					{
						if (GetCurrentActiveGear() == 0)
						{
							valueToSet = EEngineState::EngineIdle;
						}
						else
						{
							if (!IsGasPedalPressed(gasPedalMinValue + (gasPedalMaxValue - gasPedalMinValue) * 0.1f))
							{
								int currentGear = GetCurrentActiveGear();
								float minimumSpeed = 0;
								float currentEngineRPM = GetEngineRotationSpeed();
								if (currentGear > 0)
								{
									minimumSpeed = TransmissionSetup.GetMinimumSpeedForGear(currentGear, true);
								}
								else if(currentGear<0)
								{
									minimumSpeed = TransmissionSetup.GetMinimumSpeedForGear(currentGear * -1, false);
								}

								if (TransmissionSetup.instantStopOnNoGasOrClutch)
									minimumSpeed = BIG_NUMBER;

								if ((minimumSpeed - currentSpeed) > KINDA_SMALL_NUMBER && isVehicleAccelerating == false && (currentEngineRPM <= previousEngineRPM)
									&& currentInputs.currentClutchPedalValue < (clutchPedalMinValue + (clutchPedalMaxValue - clutchPedalMinValue) * 0.1f))
								{
									actionError.didEngineStall = true;
									actionError.errorReason = EActionErrorReason::SpeedTooLowForGear;
									actionError.areValuesFilled = true;

									valueToSet = EEngineState::EngineOff;
								}
								else
								{
									valueToSet = EEngineState::EngineEngaged;
								}
							}
							else
							{
								int currentGear = GetCurrentActiveGear();
								float minimumSpeed = 0;
								float currentEngineRPM = GetEngineRotationSpeed();
								if (currentGear > 2)
								{
									minimumSpeed = TransmissionSetup.GetMinimumSpeedForGear(currentGear - 2, true);
								}
								else if (currentGear < -2)
									minimumSpeed = TransmissionSetup.GetMinimumSpeedForGear(currentGear * -1 - 2, false);


								if ((minimumSpeed - currentSpeed) > 1 /*&& (currentEngineRPM < previousEngineRPM || UKismetMathLibrary::Abs(currentEngineRPM - EngineSetup.EngineIdleRPM) < .1)*/)
								{
									actionError.didEngineStall = true;
									actionError.errorReason = EActionErrorReason::SpeedTooLowForGear;
									actionError.areValuesFilled = true;
									valueToSet = EEngineState::EngineOff;
								}
								else
									valueToSet = EEngineState::EngineEngaged;
							}
						}
					}
				}
				else
				{
					actionError.didEngineStall = true;
					actionError.errorReason = EActionErrorReason::NoGasOrClutch;
					actionError.areValuesFilled = true;
					valueToSet = EEngineState::EngineOff;
				}
			}
			if (currentEngineState == EEngineState::EngineGearChangeable)
			{
				if ((GetNetFuelIntake() >= fuelValueToSustainIdleRPM) || (GetNetFuelIntake() < fuelValueToSustainIdleRPM && currentInputs.currentClutchPedalValue> (clutchPedalMaxValue - clutchPedalMinValue)*0.1f ))
				{
					if (currentInputs.currentClutchPedalValue >= clutchThresholdValue)
					{
						if (currentInputs.currentClutchPedalValue >= clutchDisengagementValue)
						{
							valueToSet = EEngineState::EngineDisengaged;
						}
						else
						{
							valueToSet = EEngineState::EngineGearChangeable;
						}
					}
					else if (currentInputs.currentClutchPedalValue < clutchThresholdValue)
					{
						if (GetCurrentActiveGear() != 0)
						{
							if( IsGasPedalPressed((gasPedalMaxValue - gasPedalMinValue) * .1f))
								valueToSet = EEngineState::EngineEngaged;
							else if((GetCurrentActiveGear()==1 || GetCurrentActiveGear() == -1 )&& previousGear==0)
								valueToSet = EEngineState::EngineOff;
							else
								valueToSet = EEngineState::EngineEngaged;


						}
						else
						{
							valueToSet = EEngineState::EngineIdle;
						}
					}
				}
				else
				{
					actionError.didEngineStall = false;
					actionError.errorReason = EActionErrorReason::NoGasOrClutch;
					actionError.areValuesFilled = true;
					/*if(GetCurrentActiveGear()<=1)
					valueToSet = EEngineState::EngineOff;*/ //will be turned off by speed instead
				}
			}
			if (currentEngineState == EEngineState::EngineDisengaged)
			{
				if ((GetNetFuelIntake() >= fuelValueToSustainIdleRPM) || (GetNetFuelIntake() < fuelValueToSustainIdleRPM && currentInputs.currentClutchPedalValue> (clutchPedalMaxValue - clutchPedalMinValue)*0.1f ))
				{
					if (currentInputs.currentClutchPedalValue >= clutchThresholdValue)
					{
						if (currentInputs.currentClutchPedalValue >= clutchDisengagementValue)
						{
							valueToSet = EEngineState::EngineDisengaged;
						}
						else
						{
							valueToSet = EEngineState::EngineGearChangeable;
						}
					}
					else if (currentInputs.currentClutchPedalValue < clutchThresholdValue)
					{
						if (GetCurrentActiveGear() != 0)
						{
							if (IsGasPedalPressed((gasPedalMaxValue - gasPedalMinValue) * .1f))
								valueToSet = EEngineState::EngineEngaged;
							else
								valueToSet = EEngineState::EngineOff;

						}
						else
						{
							valueToSet = EEngineState::EngineIdle;
						}
					}
				}
				else
				{
					actionError.didEngineStall = true;
					actionError.errorReason = EActionErrorReason::NoGasOrClutch;
					actionError.areValuesFilled = true;
					if (GetCurrentActiveGear() <= 1)
						valueToSet = EEngineState::EngineOff;
				}
			}
		}
		
		if (valueToSet != currentEngineState)
		{
			UE_LOG(LogTemp, Log, TEXT("Engine State changed to : %s"), *UEnum::GetValueAsString(valueToSet));
			vehicleStateChanged.Broadcast(valueToSet, currentEngineState);
		}
		if (valueToSet == EEngineState::EngineOff)
		{
			EndFakeRPM();
			currentEngineStartedValue = false;
			previousEngineRPM = EngineSetup.EngineIdleRPM;
			previousVehicleSpeed = 0;
			vehicleStopped.Broadcast();

			SetThrottleInput(0);
			if (derivedPtrForSimulationClass)
			{
				derivedPtrForSimulationClass->dataImpactingSimulation.isFilled = false;
				derivedPtrForSimulationClass->dataImpactingSimulation.isThrottleActive = false;
				derivedPtrForSimulationClass->dataImpactingSimulation.isEngineStarted = false;
			}

			if (vehicleSounds.useEngineSounds && IsValid(vehicleSounds.engineStopSound) && valueToSet!= currentEngineState)
			{
				PlayNewAudio( vehicleSounds.engineStopSound);
			}
		}
		if (valueToSet == EEngineState::EngineIdle)
		{
			EndFakeRPM();
			//currentGasPedalValue = gasPedalMinValue;
			//currentClutchPedalValue = clutchPedalMinValue;
			previousEngineRPM = EngineSetup.EngineIdleRPM;
			previousVehicleSpeed = 0;

			SetThrottleInput(0);
			if (derivedPtrForSimulationClass)
			{
				derivedPtrForSimulationClass->dataImpactingSimulation.isFilled = false;
				derivedPtrForSimulationClass->dataImpactingSimulation.isThrottleActive = false;
			}

			if (vehicleSounds.useEngineSounds && IsValid(vehicleSounds.engineStartupSound) && valueToSet != currentEngineState && currentEngineState == EEngineState::EngineOff)
			{

				PlayNewAudio( vehicleSounds.engineStartupSound);
			}
			else if (vehicleSounds.useEngineSounds && IsValid(vehicleSounds.engineIdleSound) && valueToSet != currentEngineState)
			{
				PlayNewAudio( vehicleSounds.engineIdleSound);
			}

		}
		if (valueToSet == EEngineState::EngineEngaged)
		{
			EndFakeRPM();
			if (vehicleSounds.useEngineSounds && !vehicleSounds.useSeparateSoundsPerGear && valueToSet != currentEngineState)
			{
				PlayNewAudio( vehicleSounds.engineEngagedSound);
			}
			else
			{

				if(vehicleSounds.engineGearWiseSound.Num()>= TransmissionSetup.GetForwardGearCount() && valueToSet != currentEngineState)
				PlayNewAudio(vehicleSounds.engineGearWiseSound[FMath::Abs(GetCurrentActiveGear())-1]);
			}
		}
		if (valueToSet == EEngineState::EngineGearChangeable || valueToSet == EEngineState::EngineDisengaged)
		{
			//StartFakeRPM();
		}
		currentEngineState = valueToSet;
		if(actionError.areValuesFilled)
		actionErrorCaused.Broadcast(actionError);

		previousEngineRPM = GetEngineRotationSpeed();

	}
}

bool UDynamicVehicleMovementComponent::ToggleBreakAssist(bool enableBreakAssist)
{
	bool inputValueUpdated = false;
	if (currentInputs.currentBreakAssistValue != enableBreakAssist)
		inputValueUpdated = true;

	if (vehicleFunctionalities.vehicleHasBrakekAssist)
	{
		if (currentEngineState == EEngineState::EngineOff)
		{
			currentInputs.currentBreakAssistValue = false;
			if(inputValueUpdated)
				brakeAssistInputChanged.Broadcast(false);
			brakeAssistInputReleased.Broadcast();
			return false;
		}
		else
		{
			currentInputs.currentBreakAssistValue = enableBreakAssist;
			if(inputValueUpdated)
				brakeAssistInputChanged.Broadcast(enableBreakAssist);
			if (!enableBreakAssist)
				brakeAssistInputReleased.Broadcast();
			return true;
		}
	}
	else
	{
		return false;
	}
}

bool UDynamicVehicleMovementComponent::AdjustFuelHandle(float fuelValue)
{
	bool inputValueUpdated = false;

	if (FMath::Abs(currentInputs.currentFuelHandleValue - fuelValue) > KINDA_SMALL_NUMBER)
		inputValueUpdated = true;

	if (vehicleFunctionalities.vehicleHasManualFuelHandle)
	{
		if (currentInputs.currentFuelHandleValue < fuelValueToSustainIdleRPM && fuelValue >= fuelValueToSustainIdleRPM)
		{
			currentInputs.currentFuelHandleValue = fuelValue;
			UpdateVehicleEngineState();
		}
		else if (currentInputs.currentFuelHandleValue >= fuelValueToSustainIdleRPM && fuelValue < fuelValueToSustainIdleRPM)
		{
			currentInputs.currentFuelHandleValue = fuelValue;

			UpdateVehicleEngineState();
		}
		else
			currentInputs.currentFuelHandleValue = fuelValue;

		if (inputValueUpdated)
			manualFuelHandleInputChanged.Broadcast(fuelValue);
		if (fuelValue < fuelValueToSustainIdleRPM)
			manualFuelHandleInputReleased.Broadcast();

		return true;
	}
	else
		return false;
}

void UDynamicVehicleMovementComponent::SetWiperMode(EWiperModes newMode)
{
	bool inputValueUpdated = false;

	if (currentInputs.currentWiperMode != newMode)
		inputValueUpdated = true;

	if (vehicleFunctionalities.vehicleHasNonFunctionalAspects)
	{
		if (IsValid(vehicleNonFunctionalAspects.leftWiper.fetchedMeshComponent) && IsValid(vehicleNonFunctionalAspects.leftWiper.fetchedMeshComponent->GetSkeletalMeshAsset()))
		{
			vehicleNonFunctionalAspects.leftWiper.fetchedMeshComponent->PlayAnimation(vehicleNonFunctionalAspects.wiperAnimation,true);
			vehicleNonFunctionalAspects.leftWiper.fetchedMeshComponent->SetPlayRate(static_cast<uint8>(newMode));

		}
		if (IsValid(vehicleNonFunctionalAspects.rightwiper.fetchedMeshComponent) && IsValid(vehicleNonFunctionalAspects.rightwiper.fetchedMeshComponent->GetSkeletalMeshAsset()))
		{
			vehicleNonFunctionalAspects.rightwiper.fetchedMeshComponent->PlayAnimation(vehicleNonFunctionalAspects.wiperAnimation, true);
			vehicleNonFunctionalAspects.rightwiper.fetchedMeshComponent->SetPlayRate(static_cast<uint8>(newMode));

		}
	
		if (inputValueUpdated)
			wiperInputChanged.Broadcast(static_cast<uint8>(newMode));
		if (inputValueUpdated && newMode == EWiperModes::Off)
			wiperInputReleased.Broadcast();
		currentInputs.currentWiperMode = newMode;
	}
}

bool UDynamicVehicleMovementComponent::PlayNewAudio(USoundBase* newSound)
{

	if(IsValid(newSound))
		currentEngineSound = UGameplayStatics::SpawnSound2D(GetWorld(), newSound);
	if (IsValid(currentEngineSound))
		return true;
	else
		return false;
}

bool UDynamicVehicleMovementComponent::PlayHorn(USoundBase* newSound, bool play, float volume)
{
	if (play)
	{
		if (IsValid(newSound))
		{
			currentHornSound = UGameplayStatics::SpawnSound2D(GetWorld(), newSound, volume);
			return true;
		}
		else
			return false;
	}
	else
	{
		if(IsValid(currentHornSound))
			currentHornSound->Stop();
		return true;
	}
}

bool UDynamicVehicleMovementComponent::PlayIndicatorSound(USoundBase* newSound, bool play, float volume)
{
	if (play)
	{
		if (IsValid(newSound))
		{
			currentIndicatorSound = UGameplayStatics::SpawnSound2D(GetWorld(), newSound, volume);
			return true;
		}
		else
			return false;
	}
	else
	{
		currentIndicatorSound->Stop();
		return true;
	}
}

bool UDynamicVehicleMovementComponent::PlayWiperSound(USoundBase* newSound, bool play, float volume)
{
	if (play)
	{
		if (IsValid(newSound))
		{
			currentWiperSound = UGameplayStatics::SpawnSound2D(GetWorld(), newSound, volume);
			return true;
		}
		else
			return false;
	}
	else
	{
		currentWiperSound->Stop();
		return true;
	}
}

bool UDynamicVehicleMovementComponent::PlayBrakeSound(USoundBase* newSound, bool play, float volume)
{
	if (play)
	{
		if (IsValid(newSound))
		{
			currentBreakSound = UGameplayStatics::SpawnSound2D(GetWorld(), newSound, volume);
			return true;
		}
		else
			return false;
	}
	else
	{
		currentBreakSound->Stop();
		return true;
	}
}

bool UDynamicVehicleMovementComponent::PlayRoadSound(USoundBase* newSound, bool play, float volume)
{
	if (play)
	{
		if (IsValid(currentRoadSound) && currentRoadSound->IsPlaying())
		{
			currentRoadSound->Stop();
		}
		if (IsValid(newSound))
		{
			currentRoadSound = UGameplayStatics::SpawnSound2D(GetWorld(), newSound, volume);
			return true;
		}
		else
			return false;
	}
	else if (IsValid(currentRoadSound) && currentRoadSound->IsPlaying())
	{
		currentRoadSound->Stop();
		return true;
	}
	return false;
}

void UDynamicVehicleMovementComponent::ChangeTransmissionSystem(bool isAutomatic)
{
	if (!TransmissionSetup.IsCurrentTransmissionModeManual())
	{
		if (isAutomatic)
		{
			return;
		}
		else
		{
			TransmissionSetup.SetTransmissionMode(true);
			SetUseAutomaticGears(false);
			if (derivedPtrForSimulationClass)
			{
				derivedPtrForSimulationClass->dataImpactingSimulation.transmissionData = TransmissionSetup;
			}
		}
	}
	else
	{
		if (isAutomatic)
		{
			TransmissionSetup.SetTransmissionMode(false);
			SetUseAutomaticGears(true);
			if (derivedPtrForSimulationClass)
			{
				derivedPtrForSimulationClass->dataImpactingSimulation.transmissionData = TransmissionSetup;
			}
		}
		else
			return;
	}
}



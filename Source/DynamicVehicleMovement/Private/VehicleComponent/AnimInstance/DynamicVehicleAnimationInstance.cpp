// Copyright Epic Games, Inc. All Rights Reserved.

/*=============================================================================
	UDynamicVehicleAnimationInstance.cpp: Single Node Tree Instance 
	Only plays one animation at a time. 
=============================================================================*/ 

#include "VehicleComponent/AnimInstance/DynamicVehicleAnimationInstance.h"
#include "VehicleComponent/DynamicVehicleMovementComponent.h"
#include "Wheels/DynamicWheel/DynamicWheel.h"
#include "WheeledVehiclePawn.h"
#include "AnimationRuntime.h"


	/////////////////////////////////////////////////////
	// UDynamicVehicleAnimationInstance
	/////////////////////////////////////////////////////


	class AWheeledVehiclePawn* UDynamicVehicleAnimationInstance::GetVehicle()
	{
		return Cast<AWheeledVehiclePawn>(GetOwningActor());
	}

	void UDynamicVehicleAnimationInstance::NativeInitializeAnimation()
	{
		// Find a wheeled movement component
		if (AActor* Actor = GetOwningActor())
		{
			if (UDynamicVehicleMovementComponent* FoundWheeledVehicleComponent = Actor->FindComponentByClass<UDynamicVehicleMovementComponent>())
			{
				SetWheeledVehicleComponent(FoundWheeledVehicleComponent);
			}
		}
	}

	FAnimInstanceProxy* UDynamicVehicleAnimationInstance::CreateAnimInstanceProxy()
	{
		return &AnimInstanceProxy;
	}

	void UDynamicVehicleAnimationInstance::DestroyAnimInstanceProxy(FAnimInstanceProxy* InProxy)
	{
	}

	/////////////////////////////////////////////////////
	//// PROXY ///
	/////////////////////////////////////////////////////

	void FDynamicVehicleAnimationInstanceProxy::SetWheeledVehicleComponent(const UDynamicVehicleMovementComponent* InWheeledVehicleComponent)
	{
		const UDynamicVehicleMovementComponent* WheeledVehicleComponent = InWheeledVehicleComponent;

		//initialize wheel data
		const int32 NumOfwheels = (WheeledVehicleComponent->IsUsingSystem1ForDifferential()?(WheeledVehicleComponent->WheelSetupsForDifferentialSystem1):(WheeledVehicleComponent->WheelSetupsForDifferentialSystem2)).Num();
		WheelInstances.Empty(NumOfwheels);
		if (NumOfwheels > 0)
		{
			WheelInstances.AddZeroed(NumOfwheels);
			// now add wheel data
			for (int32 WheelIndex = 0; WheelIndex < WheelInstances.Num(); ++WheelIndex)
			{
				FDynamicWheelAnimationData& WheelInstance = WheelInstances[WheelIndex];
				const FDynamicWheelSetup& WheelSetup = (WheeledVehicleComponent->IsUsingSystem1ForDifferential() ? (WheeledVehicleComponent->WheelSetupsForDifferentialSystem1) : (WheeledVehicleComponent->WheelSetupsForDifferentialSystem2))[WheelIndex];

				// set data
				WheelInstance.BoneName = WheelSetup.BoneName;
				WheelInstance.LocOffset = FVector::ZeroVector;
				WheelInstance.RotOffset = FRotator::ZeroRotator;
			}
		}
	}

	void FDynamicVehicleAnimationInstanceProxy::PreUpdate(UAnimInstance* InAnimInstance, float DeltaSeconds)
	{
		Super::PreUpdate(InAnimInstance, DeltaSeconds);

		const UDynamicVehicleAnimationInstance* VehicleAnimInstance = CastChecked<UDynamicVehicleAnimationInstance>(InAnimInstance);
		if (const UDynamicVehicleMovementComponent* WheeledVehicleComponent = VehicleAnimInstance->GetWheeledVehicleComponent())
		{
			for (int32 WheelIndex = 0; WheelIndex < WheelInstances.Num(); ++WheelIndex)
			{
				FDynamicWheelAnimationData& WheelInstance = WheelInstances[WheelIndex];
				if (WheeledVehicleComponent->Wheels.IsValidIndex(WheelIndex))
				{
					if (const UDynamicWheel* VehicleWheel = WheeledVehicleComponent->Wheels[WheelIndex])
					{
						if (WheelSpokeCount > 0 && ShutterSpeed > 0 && MaxAngularVelocity > SMALL_NUMBER) // employ stagecoach effect
						{
							// normalized spoke transition value
							float AngularVelocity = VehicleWheel->GetRotationAngularVelocity();
							float DegreesPerFrame = AngularVelocity / ShutterSpeed;
							float DegreesPerSpoke = 360.f / WheelSpokeCount;

							float IntegerPart = 0;
							float SpokeTransition = FMath::Modf(DegreesPerFrame / DegreesPerSpoke, &IntegerPart);
							float StagecoachEffectVelocity = FMath::Sin(SpokeTransition * TWO_PI) * MaxAngularVelocity;

							// blend
							float OffsetVelocity = FMath::Abs(AngularVelocity) - MaxAngularVelocity;
							if (OffsetVelocity < 0.f)
							{
								OffsetVelocity = 0.f;
							}

							float BlendAlpha = FMath::Clamp(OffsetVelocity / MaxAngularVelocity, 0.f, 1.f);

							float CorrectedAngularVelocity = FMath::Lerp(AngularVelocity, StagecoachEffectVelocity, BlendAlpha);
							CorrectedAngularVelocity = FMath::Clamp(CorrectedAngularVelocity, -MaxAngularVelocity, MaxAngularVelocity);

							// integrate to angular position
							float RotationDelta = CorrectedAngularVelocity * DeltaSeconds;
							WheelInstance.RotOffset.Pitch += RotationDelta;

							int ExcessRotations = (int)(WheelInstance.RotOffset.Pitch / 360.0f);
							if (FMath::Abs(ExcessRotations) > 1)
							{
								WheelInstance.RotOffset.Pitch -= ExcessRotations * 360.0f;
							}
						}
						else
						{
							WheelInstance.RotOffset.Pitch = VehicleWheel->GetRotationAngle();
						}
						WheelInstance.RotOffset.Yaw = VehicleWheel->GetSteerAngle();
						WheelInstance.RotOffset.Roll = 0.f;

						WheelInstance.LocOffset = -VehicleWheel->GetSuspensionAxis() * VehicleWheel->GetSuspensionOffset();
					}
				}
			}
		}
	}


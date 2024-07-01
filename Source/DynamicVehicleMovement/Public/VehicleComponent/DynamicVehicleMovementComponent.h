// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Physics/PhysicsInterfaceCore.h"
#include "UObject/ObjectMacros.h"
#include "ChaosVehicleMovementComponent.h"
#include "Curves/CurveFloat.h"
#include "VehicleUtility.h"
#include "Chaos/PBDSuspensionConstraints.h"
#include "PhysicsProxy/SingleParticlePhysicsProxyFwd.h"
#include "Components/LightComponent.h"
#include "DynamicVehicleMovementComponent.generated.h"

#if VEHICLE_DEBUGGING_ENABLED
PRAGMA_DISABLE_OPTIMIZATION
#endif
USTRUCT(BlueprintType)
struct DYNAMICVEHICLEMOVEMENT_API FDynamicWheelSetup
{
	GENERATED_BODY()

	// The wheel class to use
	UPROPERTY(EditAnywhere, Category = WheelSetup)
	TSubclassOf<UDynamicWheel> WheelClass;

	// Bone name on mesh to create wheel at
	//UPROPERTY(EditAnywhere, Category = WheelSetup)
	//FName SteeringBoneName;

	// Bone name on mesh to create wheel at
	UPROPERTY(EditAnywhere, Category = WheelSetup)
	FName BoneName;

	// Additional offset to give the wheels for this axle.
	UPROPERTY(EditAnywhere, Category = WheelSetup)
	FVector AdditionalOffset;

	FDynamicWheelSetup();
};

struct DYNAMICVEHICLEMOVEMENT_API FDynamicWheeledVehicleDebugParams
{
	bool ShowWheelCollisionNormal = false;
	bool ShowSuspensionRaycasts = false;
	bool ShowSuspensionLimits = false;
	bool ShowWheelForces = false;
	bool ShowSuspensionForces = false;
	bool ShowBatchQueryExtents = false;
	bool ShowRaycastComponent = false;
	bool ShowRaycastMaterial = false;
	int TraceTypeOverride = 0;

	bool DisableSuspensionForces = false;
	bool DisableFrictionForces = false;
	bool DisableRollbarForces = false;
	bool DisableConstraintSuspension = false;

	float ThrottleOverride = 0.f;
	float SteeringOverride = 0.f;

	bool ResetPerformanceMeasurements = false;

	float OverlapTestExpansionXY = 100.f;
	float OverlapTestExpansionZ = 50.f;
};

/**
 * There is too much information for one screen full of debug data, so sub-pages of information are available
 * Advance through pages using p.Vehicles.NextDebugPage|p.Vehicles.PrevDebugPage which can be hooked
 * up to the keyboard or a controller in blueprint using execCommand
 */
enum class EDynamicDebugPages 
{
	BasicPage = 0,
	PerformancePage,
	SteeringPage,
	FrictionPage,
	SuspensionPage,
	TransmissionPage,

	MaxDebugPages	// keep as last value
};

UENUM()
enum class EDynamicVehicleDifferential : uint8
{
	Undefined,
	AllWheelDrive,
	FrontWheelDrive,
	RearWheelDrive,
};

UENUM()
enum class EEngineState: uint8
{
	//The vehicle is not running
	EngineOff,
	//Vehicle is running and is in neutral gear
	EngineIdle,
	//Vehicle is running and is in a driveable gear
	EngineEngaged,
	//Vehicle is running, in driveable gear, but clutch is pressed enough to allow gear change while vehicle is driveable
	EngineGearChangeable,
	//Vehicle is running but clutch is pressed fully, disengaging the engine
	EngineDisengaged
};

/**
 * Structure containing information about the status of a single wheel of the vehicle.
 */
USTRUCT(BlueprintType)
struct DYNAMICVEHICLEMOVEMENT_API FDynamicWheelStatus
{
	GENERATED_BODY()

	/** This wheel is in contact with the ground */
	UPROPERTY()
	bool bInContact;

	/** Wheel contact point */
	UPROPERTY()
	FVector ContactPoint;

	/** Wheel contact location */
	UPROPERTY()
	FVector HitLocation;

	/** Material that wheel is in contact with */
	UPROPERTY()
	TWeakObjectPtr<class UPhysicalMaterial> PhysMaterial;

	/** Normalized suspension length at this wheel */
	UPROPERTY()
	float NormalizedSuspensionLength;

	/** Spring Force that is occurring at wheel suspension */
	UPROPERTY()
	float SpringForce;

	/** Slip angle at the wheel - difference between wheel local direction and velocity at wheel */
	UPROPERTY()
	float SlipAngle;

	/** Is the wheel slipping */
	UPROPERTY()
	bool bIsSlipping;

	/** Magnitude of slippage of wheel, difference between wheel speed and ground speed */
	UPROPERTY()
	float SlipMagnitude;


	/** Is the wheel skidding */
	UPROPERTY()
	bool bIsSkidding;

	/** Magnitude of skid */
	UPROPERTY()
	float SkidMagnitude;

	/** Direction of skid, i.e. normalized direction */
	UPROPERTY()
	FVector SkidNormal;

	/** Drive torque currently applied at wheel */
	UPROPERTY()
	float DriveTorque;

	/** Brake torque currently applied at wheel */
	UPROPERTY()
	float BrakeTorque;

	/** Is the ABS currently engaged - useful for audio Q's */
	UPROPERTY()
	bool bABSActivated;

	FDynamicWheelStatus()
	{
		Init();
	};

	explicit FDynamicWheelStatus(EForceInit InInit)
	{
		Init();
	};

	FDynamicWheelStatus(ENoInit NoInit)
	{
		bIsValid = false;
	};

	void Init()
	{
		SlipAngle = 0.0f;
		bInContact = false;
		bIsSlipping = false;
		bIsSkidding = false;
		SlipMagnitude = 0.f;
		SkidMagnitude = 0.f;
		NormalizedSuspensionLength = 1.f;
		SpringForce = 0.f;
		SkidNormal = FVector::ZeroVector;
		ContactPoint = FVector::ZeroVector;
		HitLocation = FVector::ZeroVector;
		bIsValid = false;
		bABSActivated = false;
		DriveTorque = 0.0f;
		BrakeTorque = 0.0f;
	};

	FString ToString() const;

	bool bIsValid;
};

USTRUCT()
struct DYNAMICVEHICLEMOVEMENT_API FDynamicVehicleDifferentialConfig
{
	GENERATED_USTRUCT_BODY()

	FDynamicVehicleDifferentialConfig()
	{
		InitDefaults();
	}

	/** Type of differential. */
	UPROPERTY(EditAnywhere)
	EDynamicVehicleDifferential DifferentialTypeForSystem1;

	/** Ratio of torque split between front and rear (<0.5 means more to front, >0.5 means more to rear, works only with 4W type) */
	UPROPERTY(EditAnywhere, meta = (ClampMin = "0.0", UIMin = "0.0", ClampMax = "1.0", UIMax = "1.0", EditCondition = "DifferentialTypeForSystem1==EDynamicVehicleDifferential::AllWheelDrive"))
	float FrontRearSplitForSystem1;

	/** Type of differential */
	UPROPERTY(EditAnywhere)
	EDynamicVehicleDifferential DifferentialTypeForSystem2;

	/** Ratio of torque split between front and rear (<0.5 means more to front, >0.5 means more to rear, works only with 4W type) */
	UPROPERTY(EditAnywhere, meta = (ClampMin = "0.0", UIMin = "0.0", ClampMax = "1.0", UIMax = "1.0", EditCondition = "DifferentialTypeForSystem2==EDynamicVehicleDifferential::AllWheelDrive"))
	float FrontRearSplitForSystem2;

	const Chaos::FSimpleDifferentialConfig& GetPhysicsDifferentialConfig(bool getSystem1=true)
	{
		FillDifferentialSetup();
		if(getSystem1)
			return PDifferentialConfigforSystem1;
		else
			return PDifferentialConfigforSystem2;
	}

	void InitDefaults()
	{
		DifferentialTypeForSystem1 = EDynamicVehicleDifferential::RearWheelDrive;
		FrontRearSplitForSystem1 = 0.5f;
		DifferentialTypeForSystem2 = EDynamicVehicleDifferential::AllWheelDrive;
		FrontRearSplitForSystem2 = 0.5f;
	}

	void FillDifferentialSetup()
	{
		PDifferentialConfigforSystem1.DifferentialType = static_cast<Chaos::EDifferentialType>(this->DifferentialTypeForSystem1);
		PDifferentialConfigforSystem1.FrontRearSplit = this->FrontRearSplitForSystem1;

		PDifferentialConfigforSystem2.DifferentialType = static_cast<Chaos::EDifferentialType>(this->DifferentialTypeForSystem2);
		PDifferentialConfigforSystem2.FrontRearSplit = this->FrontRearSplitForSystem2;
	}

	EDynamicVehicleDifferential GetDifferentialTypefromActiveSystem(bool getSystem1 = true)
	{
		if (getSystem1)
		{
			return DifferentialTypeForSystem1;
		}
		else
		{
			return DifferentialTypeForSystem2;
		}
	}

	float GetDifferentialSplitRatiofromActiveSystem(bool getSystem1 = true)
	{
		if (getSystem1)
		{
			return FrontRearSplitForSystem1;
		}
		else
		{
			return FrontRearSplitForSystem2;
		}
	}


	Chaos::FSimpleDifferentialConfig PDifferentialConfigforSystem1;
	Chaos::FSimpleDifferentialConfig PDifferentialConfigforSystem2;
};

USTRUCT()
struct DYNAMICVEHICLEMOVEMENT_API FDynamicVehicleEngineConfig
{
	GENERATED_USTRUCT_BODY()

	FDynamicVehicleEngineConfig()
	{
		InitDefaults();
	}

	/** Torque [Normalized 0..1] for a given RPM */
	UPROPERTY(EditAnywhere, Category = Setup)
	FRuntimeFloatCurve TorqueCurve;

	/** Max Engine Torque (Nm) is multiplied by TorqueCurve */
	UPROPERTY(EditAnywhere, Category = Setup)
	float MaxTorque;

	/** Maximum revolutions per minute of the engine */
	UPROPERTY(EditAnywhere, Category = Setup, meta = (ClampMin = "0.01", UIMin = "0.01"))
	float MaxRPM;

	/** Idle RMP of engine then in neutral/stationary */
	UPROPERTY(EditAnywhere, Category = Setup, meta = (ClampMin = "0.01", UIMin = "0.01"))
	float EngineIdleRPM;

	/** Braking effect from engine, when throttle released */
	UPROPERTY(EditAnywhere, Category = Setup)
	float EngineBrakeEffect;

	/** Affects how fast the engine RPM speed up */
	UPROPERTY(EditAnywhere, Category = Setup, meta = (ClampMin = "0.01", UIMin = "0.01"))
	float EngineRevUpMOI;

	/** Affects how fast the engine RPM slows down */
	UPROPERTY(EditAnywhere, Category = Setup, meta = (ClampMin = "0.01", UIMin = "0.01"))
	float EngineRevDownRate;

	const Chaos::FSimpleEngineConfig& GetPhysicsEngineConfig()
	{
		FillEngineSetup();
		return PEngineConfig;
	}

	void InitDefaults()
	{
		MaxTorque = 300.0f;
		MaxRPM = 4500.0f;
		EngineIdleRPM = 1200.0f;
		EngineBrakeEffect = 0.05f;
		EngineRevUpMOI = 5.0f;
		EngineRevDownRate = 600.0f;
	}

	float GetTorqueFromRPM(float EngineRPM)
	{
		// The source curve does not need to be normalized, however we are normalizing it when it is passed on,
		// since it's the MaxRPM and MaxTorque values that determine the range of RPM and Torque
		float MinVal = 0.f, MaxVal = 0.f;
		this->TorqueCurve.GetRichCurveConst()->GetValueRange(MinVal, MaxVal);
		return TorqueCurve.GetRichCurve()->Eval(EngineRPM) / MaxVal * MaxTorque;
	}
private:

	void FillEngineSetup()
	{
		// The source curve does not need to be normalized, however we are normalizing it when it is passed on,
		// since it's the MaxRPM and MaxTorque values that determine the range of RPM and Torque
		PEngineConfig.TorqueCurve.Empty();
		float NumSamples = 20;
		for (float X = 0; X <= this->MaxRPM; X += (this->MaxRPM / NumSamples))
		{
			float MinVal = 0.f, MaxVal = 0.f;
			this->TorqueCurve.GetRichCurveConst()->GetValueRange(MinVal, MaxVal);
			float Y = this->TorqueCurve.GetRichCurveConst()->Eval(X) / MaxVal;
			PEngineConfig.TorqueCurve.AddNormalized(Y);
		}
		PEngineConfig.MaxTorque = this->MaxTorque;
		PEngineConfig.MaxRPM = this->MaxRPM;
		PEngineConfig.EngineIdleRPM = this->EngineIdleRPM;
		PEngineConfig.EngineBrakeEffect = this->EngineBrakeEffect;
		PEngineConfig.EngineRevUpMOI = this->EngineRevUpMOI;
		PEngineConfig.EngineRevDownRate = this->EngineRevDownRate;
	}

	Chaos::FSimpleEngineConfig PEngineConfig;

};

USTRUCT(BlueprintType)
struct DYNAMICVEHICLEMOVEMENT_API FHighLowGearCombo
{
	GENERATED_USTRUCT_BODY()

	FHighLowGearCombo()
	{
		HighRatio = 0;
		LowRatio = 0;
	}

	FHighLowGearCombo(float singleValue)
	{
		HighRatio = singleValue;
		LowRatio = singleValue;
	}

	FHighLowGearCombo(float highValue, float lowValue)
	{
		HighRatio = highValue;
		LowRatio = lowValue;
	}


	FHighLowGearCombo operator*(float number) const {
		return { HighRatio * number, LowRatio * number };
	}

	
	//Gear ratio to use when in High Ratio mode
	UPROPERTY(EditAnywhere)
	float HighRatio = 0; 
	//Gear ratio to use when in Low Ratio mode
	UPROPERTY(EditAnywhere)
	float LowRatio = 0; 
	//Minimum speed is used to calculate engine stall condition when niether throttle nor clutch is pressed appropriately.
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = Setup)
	float MinimumSpeed = 0; 
	//Max Speed gear can reach
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = Setup)
	float MaximumSpeed = -1;

};

USTRUCT(BlueprintType)
struct DYNAMICVEHICLEMOVEMENT_API FSingularGearCombo
{
	GENERATED_USTRUCT_BODY()

	FSingularGearCombo()
	{
		Ratio = 0;
	}

	FSingularGearCombo(float singleValue)
	{
		Ratio = singleValue;
	}


	FSingularGearCombo operator*(float number) const {
		return { Ratio * number};
	}


	//Gear ratio to use
	UPROPERTY(EditAnywhere)
	float Ratio = 0;
	//Minimum speed is used to calculate engine stall condition when niether throttle nor clutch is pressed appropriately.
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = Setup)
	float MinimumSpeed = 0;
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = Setup)
	float MaximumSpeed = -1;

};

USTRUCT()
struct DYNAMICVEHICLEMOVEMENT_API FDynamicVehicleTransmissionConfig
{
	GENERATED_USTRUCT_BODY()

	FDynamicVehicleTransmissionConfig()
	{
		InitDefaults();
	}

	friend class UDynamicleWheel;

	/** Whether to use automatic transmission */
	UPROPERTY(/*EditAnywhere, Category = VehicleSetup, meta = (DisplayName = "Automatic Transmission")*/)
	bool bUseAutomaticGears;

	UPROPERTY(/*EditAnywhere, Category = VehicleSetup, meta = (DisplayName = "Automatic Reverse")*/)
	bool bUseAutoReverse;

	UPROPERTY()
	bool bUseHighLowRatios = true;

	/** The final gear ratio multiplies the transmission gear ratios.*/
	UPROPERTY(EditAnywhere, AdvancedDisplay/*, Category = Setup*/)
	float FinalRatio;

	/** Forward gear ratios */
	UPROPERTY(EditAnywhere/*, Category = Setup*/, AdvancedDisplay, meta = (EditCondition = "bUseAutomaticGears!=true&&bUseHighLowRatios", EditConditionHides))
	TArray<FHighLowGearCombo> ForwardGearRatios;

	/** Reverse gear ratio(s) */
	UPROPERTY(EditAnywhere, AdvancedDisplay/*, Category = Setup*/, meta = (EditCondition = "bUseAutomaticGears!=true&&bUseHighLowRatios", EditConditionHides))
	TArray<FHighLowGearCombo> ReverseGearRatios;

	/** Forward gear ratios */
	UPROPERTY(EditAnywhere/*, Category = Setup*/, AdvancedDisplay, meta = (EditCondition = "bUseAutomaticGears!=true&&!bUseHighLowRatios", EditConditionHides, DisplayName = "Forward Gear Ratios"))
	TArray<FSingularGearCombo> ForwardGearRatiosSingular;

	/** Reverse gear ratio(s) */
	UPROPERTY(EditAnywhere, AdvancedDisplay/*, Category = Setup*/, meta = (EditCondition = "bUseAutomaticGears!=true&&!bUseHighLowRatios", EditConditionHides, DisplayName = "Reverse Gear Ratios"))
	TArray<FSingularGearCombo> ReverseGearRatiosSingular;

	/** Forward gear ratios for automatic transmission */
	UPROPERTY(EditAnywhere/*, Category = Setup*/, AdvancedDisplay, meta = (EditCondition = "bUseAutomaticGears==true", EditConditionHides, DisplayName = "Forward Gear Ratios"))
	TArray<float> ForwardGearRatiosAutomatic;

	/** Reverse gear ratio(s) for automatic transmission*/
	UPROPERTY(EditAnywhere, AdvancedDisplay/*, Category = Setup*/, meta = (EditCondition = "bUseAutomaticGears==true", EditConditionHides, DisplayName = "Reverse Gear Ratios"))
	TArray<float> ReverseGearRatiosAutomatic;

	/** Engine Revs at which gear up change ocurrs */
	UPROPERTY(EditAnywhere, meta = (ClampMin = "0.0", UIMin = "0.0", ClampMax = "50000.0", UIMax = "50000.0", EditCondition = "bUseAutomaticGears==true")/*, Category = Setup*/)
	float ChangeUpRPM;

	/** Engine Revs at which gear down change ocurrs */
	UPROPERTY(EditAnywhere, meta = (ClampMin = "0.0", UIMin = "0.0", ClampMax = "50000.0", UIMax = "50000.0", EditCondition = "bUseAutomaticGears==true")/*, Category = Setup*/)
	float ChangeDownRPM;

	/** Time it takes to switch gears (seconds) */
	UPROPERTY(EditAnywhere/*, Category = Setup*/, meta = (ClampMin = "0.0", UIMin = "0.0"))
	float GearChangeTime;

	/** Mechanical frictional losses mean transmission might operate at 0.94 (94% efficiency) */
	UPROPERTY(EditAnywhere, AdvancedDisplay/*, Category = Setup*/)
	float TransmissionEfficiency;

	const Chaos::FSimpleTransmissionConfig& GetPhysicsTransmissionConfig()
	{
		FillTransmissionSetup();
		return PTransmissionConfig;
	}

	void InitDefaults()
	{
		bUseAutomaticGears = false;
		bUseAutoReverse = false;
		bUseHighLowRatios = true;
		FinalRatio = 3.08f;

		ForwardGearRatios.Add(2.85f);
		ForwardGearRatios.Add(2.02f);
		ForwardGearRatios.Add(1.35f);
		ForwardGearRatios.Add(1.0f);
		ReverseGearRatios.Add(2.86f);

		ForwardGearRatiosSingular.Add(2.85f);
		ForwardGearRatiosSingular.Add(2.02f);
		ForwardGearRatiosSingular.Add(1.35f);
		ForwardGearRatiosSingular.Add(1.0f);
		ReverseGearRatiosSingular.Add(2.86f);

		ForwardGearRatiosAutomatic.Add(2.85f);
		ForwardGearRatiosAutomatic.Add(2.02f);
		ForwardGearRatiosAutomatic.Add(1.35f);
		ForwardGearRatiosAutomatic.Add(1.0f);
		ReverseGearRatiosAutomatic.Add(2.86f);

		ChangeUpRPM = 4500.0f;
		ChangeDownRPM = 2000.0f;
		GearChangeTime = 0.4f;

		TransmissionEfficiency = 0.9f;
	}

	FHighLowGearCombo GetGearRatio(int32 InGear) const
	{
		if (InGear > 0) // a forwards gear
		{
			return ForwardGearRatios[InGear - 1] * FinalRatio;
		}
		else if (InGear < 0) // a reverse gear
		{
			return ReverseGearRatios[FMath::Abs(InGear) - 1] * FinalRatio * -1;
		}
		else
		{
			return 0.f; // neutral has no ratio
		}
	}

	FSingularGearCombo GetGearRatioSingular(int32 InGear) const
	{
		if (InGear > 0) // a forwards gear
		{
			return ForwardGearRatiosSingular[InGear - 1] * FinalRatio;
		}
		else if (InGear < 0) // a reverse gear
		{
			return ReverseGearRatiosSingular[FMath::Abs(InGear) - 1] * FinalRatio * -1;
		}
		else
		{
			return 0.f; // neutral has no ratio
		}
	}

	float GetGearRatioAutomatic(int32 InGear) const
	{
		if (InGear > 0) // a forwards gear
		{
			return ForwardGearRatiosAutomatic[InGear - 1] * FinalRatio;
		}
		else if (InGear < 0) // a reverse gear
		{
			return ReverseGearRatiosAutomatic[FMath::Abs(InGear) - 1] * FinalRatio * -1;
		}
		else
		{
			return 0.f; // neutral has no ratio
		}
	}

	float GetMinimumSpeedForGear(int gearNum, bool forwardRatios) const
	{
		if (bUseAutomaticGears)
			return 0.0f;
		else if (bUseHighLowRatios)
		{
			if(forwardRatios)
				return ForwardGearRatios[gearNum].MinimumSpeed;
			else
				return ReverseGearRatios[gearNum].MinimumSpeed;
		}
		else
		{
			if (forwardRatios)
				return ForwardGearRatiosSingular[gearNum].MinimumSpeed;
			else
				return ReverseGearRatiosSingular[gearNum].MinimumSpeed;
		}
	}

	float GetMaximumSpeedForGear(int gearNum, bool forwardRatios) const
	{
		if (bUseAutomaticGears)
			return 0.0f;
		else if (bUseHighLowRatios)
		{
			if (forwardRatios)
				return ForwardGearRatios[gearNum].MaximumSpeed;
			else
				return ReverseGearRatios[gearNum].MaximumSpeed;
		}
		else
		{
			if (forwardRatios)
				return ForwardGearRatiosSingular[gearNum].MaximumSpeed;
			else
				return ReverseGearRatiosSingular[gearNum].MaximumSpeed;
		}
	}

	void SetTransferCaseModifier(float inRatio)
	{
		transferCaseModifer = inRatio;
	}
private:

	float transferCaseModifer = 1;

	void FillTransmissionSetup()
	{
		PTransmissionConfig.TransmissionType = this->bUseAutomaticGears ? Chaos::ETransmissionType::Automatic : Chaos::ETransmissionType::Manual;
		PTransmissionConfig.AutoReverse = this->bUseAutoReverse;
		PTransmissionConfig.ChangeUpRPM = this->ChangeUpRPM;
		PTransmissionConfig.ChangeDownRPM = this->ChangeDownRPM;
		PTransmissionConfig.GearChangeTime = this->GearChangeTime;
		PTransmissionConfig.FinalDriveRatio = this->FinalRatio* transferCaseModifer;
		PTransmissionConfig.ForwardRatios.Reset();
		PTransmissionConfig.TransmissionEfficiency = this->TransmissionEfficiency;

		if (bUseAutomaticGears)
		{
			for (float Ratio : ForwardGearRatiosAutomatic)
			{
				PTransmissionConfig.ForwardRatios.Add(Ratio);
			}
			PTransmissionConfig.ReverseRatios.Reset();
			for (float Ratio : ReverseGearRatiosAutomatic)
			{
				PTransmissionConfig.ReverseRatios.Add(Ratio);
			}


		}
		else
		{

			
			if (bUseHighLowRatios)
			{
				TArray<float> forwardRatios;
				GetSingularFloatArrayFromHighLowArray(this->ForwardGearRatios, forwardRatios);
				for (float Ratio : forwardRatios)
				{
					PTransmissionConfig.ForwardRatios.Add(Ratio);
				}

				TArray<float> reverseRatios;
				GetSingularFloatArrayFromHighLowArray(this->ForwardGearRatios, reverseRatios);
				PTransmissionConfig.ReverseRatios.Reset();
				for (float Ratio : reverseRatios)
				{
					PTransmissionConfig.ReverseRatios.Add(Ratio);
				}
			}
			else
			{
				for (FSingularGearCombo gearInfo : ForwardGearRatiosSingular)
				{
					PTransmissionConfig.ForwardRatios.Add(gearInfo.Ratio);
				}
				PTransmissionConfig.ReverseRatios.Reset();
				for (FSingularGearCombo gearInfo : ReverseGearRatiosSingular)
				{
					PTransmissionConfig.ReverseRatios.Add(gearInfo.Ratio);
				}
			}
		}
	}

	void GetSingularFloatArrayFromHighLowArray(TArray<FHighLowGearCombo> highLowComboArray, TArray<float> &outSignularArray)
	{
		outSignularArray.Empty();
		for (int i = 0; i < highLowComboArray.Num(); i++)
		{
			outSignularArray.Add(highLowComboArray[i].HighRatio);
			outSignularArray.Add(highLowComboArray[i].LowRatio);
		}
	}

	Chaos::FSimpleTransmissionConfig PTransmissionConfig;

};

/** Single angle : both wheels steer by the same amount
 *  AngleRatio   : outer wheels on corner steer less than the inner ones by set ratio
 *  Ackermann	 : Ackermann steering principle is applied */
UENUM()
enum class EDynamicSteeringType : uint8
{
	SingleAngle,
	AngleRatio,
	Ackermann,
};

UENUM()
enum class ETransferCasePosition : int8
{
	LowRatio = -1,	//Higher Torque, Lower Speed
	Neutral = 0,	//No Movement
	HighRatio = 1	//Higher Speed, Lower Torque
};

USTRUCT()
struct DYNAMICVEHICLEMOVEMENT_API FDynamicVehicleSteeringConfig
{
	GENERATED_USTRUCT_BODY()

	FDynamicVehicleSteeringConfig()
	{
		InitDefaults();
	}

	/** Single angle : both wheels steer by the same amount
	 *  AngleRatio   : outer wheels on corner steer less than the inner ones by set ratio
	 *  Ackermann	 : Ackermann steering principle is applied */
	UPROPERTY(EditAnywhere/*, Category = SteeringSetup*/)
	EDynamicSteeringType SteeringType;

	/** Only applies when AngleRatio is selected */
	UPROPERTY(EditAnywhere/*, Category = SteeringSetup*/, meta = (EditCondition = "SteeringType==EDynamicSteeringType::AngleRatio"))
	float AngleRatio;

	UPROPERTY(EditAnywhere/*, Category = SteeringSetup*/)
	//Whether to use steer angle mentioned in wheels, or the one mentioned inside coponent
	bool useOverrideSteerAngle = false; 

	UPROPERTY(EditAnywhere/*, Category = SteeringSetup*/, meta = (EditCondition = "useOverrideSteerAngle"))
	//Wheel turn angle to override the angle mentioned in wheels
	float overrideSteerAngle = 15;

	/** Maximum steering versus forward speed (MPH) */
	UPROPERTY(EditAnywhere/*, Category = SteeringSetup*/)
	FRuntimeFloatCurve SteeringCurve;


	const Chaos::FSimpleSteeringConfig& GetPhysicsSteeringConfig(FVector2D WheelTrackDimensions)
	{
		FillSteeringSetup(WheelTrackDimensions);
		return PSteeringConfig;
	}

	void InitDefaults()
	{
		SteeringType = EDynamicSteeringType::AngleRatio;
		AngleRatio = 0.7f;

		// Init steering speed curve
		FRichCurve* SteeringCurveData = SteeringCurve.GetRichCurve();
		SteeringCurveData->AddKey(0.f, 1.f);
		SteeringCurveData->AddKey(20.f, 0.8f);
		SteeringCurveData->AddKey(60.f, 0.4f);
		SteeringCurveData->AddKey(120.f, 0.3f);
	}

private:

	void FillSteeringSetup(FVector2D WheelTrackDimensions)
	{

		PSteeringConfig.SteeringType = (Chaos::ESteerType)this->SteeringType;
		PSteeringConfig.AngleRatio = AngleRatio;

		float MinValue = 0.f, MaxValue = 1.f;
		this->SteeringCurve.GetRichCurveConst()->GetValueRange(MinValue, MaxValue);
		float MaxX = this->SteeringCurve.GetRichCurveConst()->GetLastKey().Time;
		PSteeringConfig.SpeedVsSteeringCurve.Empty();
		float NumSamples = 20;
		for (float X = 0; X <= MaxX; X += (MaxX / NumSamples))
		{
			float Y = this->SteeringCurve.GetRichCurveConst()->Eval(X) / MaxValue;
			PSteeringConfig.SpeedVsSteeringCurve.Add(FVector2D(X, Y));
		}

		PSteeringConfig.TrackWidth = WheelTrackDimensions.Y;
		PSteeringConfig.WheelBase = WheelTrackDimensions.X;
	}

	Chaos::FSimpleSteeringConfig PSteeringConfig;

};

USTRUCT(BlueprintType)
struct DYNAMICVEHICLEMOVEMENT_API FDyamicTransferCaseConfig 
{
	GENERATED_BODY()


	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Transfer Case System", meta = (DisplayName = "Transfer Case Low Ratio", AllowPrivateAccess = "true"))
	//Low Ratio for Transfer Case
	float transferCaseLowRatio = 1.662;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Transfer Case System", meta = (DisplayName = "Transfer Case High Ratio", AllowPrivateAccess = "true"))
	//High Ratio for Transfer Case 
	float transferCaseHighRatio = 0.917;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Transfer Case System", meta = (DisplayName = "Transfer Case Starting Position", AllowPrivateAccess = "true"))
	ETransferCasePosition transferCasePosition = ETransferCasePosition::HighRatio;

	bool isTransferCaseActive = true;

	float GetTransferCaseRatio()
	{
		float retValue = 0;
		if (isTransferCaseActive)
		{
			switch (transferCasePosition)
			{
			case ETransferCasePosition::LowRatio:
				retValue = transferCaseLowRatio;
				break;
			case ETransferCasePosition::Neutral:
				retValue = 0;
				break;
			case ETransferCasePosition::HighRatio:
				retValue = transferCaseHighRatio;
				break;
			}
		}
		else
		{
			retValue = 1;
		}
		return retValue;
	}

};

/** Commonly used Wheel state - evaluated once used wherever required for that frame */
struct DYNAMICVEHICLEMOVEMENT_API FDynamicWheelState
{
	void Init(int NumWheels)
	{
		WheelLocalLocation.Init(FVector::ZeroVector, NumWheels);
		WheelWorldLocation.Init(FVector::ZeroVector, NumWheels);
		WorldWheelVelocity.Init(FVector::ZeroVector, NumWheels);
		LocalWheelVelocity.Init(FVector::ZeroVector, NumWheels);
		Trace.SetNum(NumWheels);
		TraceResult.SetNum(NumWheels);
	}

	/** Commonly used Wheel state - evaluated once used wherever required for that frame */
	void CaptureState(int WheelIdx, const FVector& WheelOffset, const FBodyInstance* TargetInstance);
	void CaptureState(int WheelIdx, const FVector& WheelOffset, const Chaos::FRigidBodyHandle_Internal* Handle);
	void CaptureState(int WheelIdx, const FVector& WheelOffset, const Chaos::FRigidBodyHandle_Internal* VehicleHandle, const FVector& ContactPoint, const Chaos::FRigidBodyHandle_Internal* SurfaceHandle);
	static FVector GetVelocityAtPoint(const Chaos::FRigidBodyHandle_Internal* Rigid, const FVector& InPoint);

	TArray<FVector> WheelLocalLocation;	/** Current Location Of Wheels In Local Coordinates */
	TArray<FVector> WheelWorldLocation;	/** Current Location Of Wheels In World Coordinates */
	TArray<FVector> WorldWheelVelocity; /** Current velocity at wheel location In World Coordinates - combined linear and angular */
	TArray<FVector> LocalWheelVelocity; /** Local velocity of Wheel */
	TArray<Chaos::FSuspensionTrace> Trace;
	TArray<FHitResult> TraceResult;
};

USTRUCT(BlueprintType)
struct DYNAMICVEHICLEMOVEMENT_API FDynamicFunctionalities
{
	GENERATED_BODY()

	FDynamicFunctionalities()
	{
		vehicleHasBreakAssist = true;
		vehicleHasMultipleDifferentials = true;
		vehicleHasHighLowGears = true;
		vehicleHasManualFuelHandle = true;
	}

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Functionalities", meta = (DisplayName = "Vehicle has Break Assist?", AllowPrivateAccess = "true"))
	//Does Vehicle have break assist capabilities?
	bool vehicleHasBreakAssist = true;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Functionalities", meta = (DisplayName = "Vehicle has Changeable Differential Systems?", AllowPrivateAccess = "true"))
	//Does Vehicle have capability to change between the 2 differentials?
	bool vehicleHasMultipleDifferentials = true;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Functionalities", meta = (DisplayName = "Vehicle has Changeable Gear Ratios?", AllowPrivateAccess = "true", EditCondition = "isVehicleAutomaticTransmission"))
	//Does Vehicle have high and low gear ratios for each gear? 
	bool vehicleHasHighLowGears = true;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Functionalities", meta = (DisplayName = "Vehicle has Manual Fuel Handle?", AllowPrivateAccess = "true", EditCondition = "isVehicleAutomaticTransmission"))
	//Does Vehicle have high and low gear ratios for each gear? 
	bool vehicleHasManualFuelHandle = true;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Functionalities", meta = (DisplayName = "Vehicle has Transfer Case?", AllowPrivateAccess = "true", EditCondition = "isVehicleAutomaticTransmission"))
	//Does Vehicle have specific transfer case with differing ratios?
	bool vehicleHasTransferCase = true;

	void SetTransmissionNature(bool isAutomatic = true)
	{
		isVehicleAutomaticTransmission = !isAutomatic;
	}
private:
	UPROPERTY()
	bool isVehicleAutomaticTransmission = false;
};

USTRUCT()
struct DYNAMICVEHICLEMOVEMENT_API FDynamicSimulationData
{
	GENERATED_BODY()

	UPROPERTY()
	bool isFilled = false;

	UPROPERTY()
	bool isThrottleActive = false;

	UPROPERTY()
	bool isBreakAssistActive = false;

	UPROPERTY()
	float netFuelIntakeValue = 0;

	UPROPERTY()
	float netFuelIntakeMinRange = 0;

	UPROPERTY()
	float netFuelIntakeMaxRange = 100;

	UPROPERTY()
	FDynamicVehicleEngineConfig engineData;

	UPROPERTY()
	FDynamicVehicleTransmissionConfig transmissionData;

	UPROPERTY()
	FDynamicFunctionalities vehicleFunctionalities;


	UPROPERTY()
	float vehicleCurrentSpeed = 0;

	void FillData(float intake, float currentSpeed = 0, bool isThrottle = false, bool isBreakAssist = false)
	{
		isFilled = true;
		isThrottleActive = isThrottle;
		netFuelIntakeValue = FMath::Clamp(intake, netFuelIntakeMinRange, netFuelIntakeMaxRange);
		vehicleCurrentSpeed = currentSpeed;
		isBreakAssistActive = isBreakAssist;
	}

	FDynamicSimulationData()
	{
		isFilled = false;
		netFuelIntakeValue = 0;
	}

	FDynamicSimulationData(float intake, float minRangeIntake, float maxRangeIntake, FDynamicVehicleTransmissionConfig transmissionConfig, FDynamicVehicleEngineConfig engineConfig, FDynamicFunctionalities functionalityConfig, bool settingRangesOnly = false)
	{
		if (!settingRangesOnly)
			isFilled = true;
		else
			isFilled = false;

		netFuelIntakeValue = intake;
		netFuelIntakeMinRange = minRangeIntake;
		netFuelIntakeMaxRange = maxRangeIntake;
		engineData = engineConfig;
		transmissionData = transmissionConfig;
		vehicleFunctionalities = functionalityConfig;

	}

};

USTRUCT(BlueprintType)
//all inputs for vehicle bundled together
struct DYNAMICVEHICLEMOVEMENT_API FDynamicInputData 
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly,  meta = (DisplayName = "Gas Pedal Current Value", AllowPrivateAccess = "true"))
	//gas pedal value ranges between 0 and 100 by default
	float currentGasPedalValue = 0;
	UPROPERTY(BlueprintReadOnly,  meta = (DisplayName = "Break Pedal Current Value", AllowPrivateAccess = "true"))
	//break pedal value ranges between 0 and 100 by default
	float currentBreakPedalValue = 0;
	UPROPERTY(BlueprintReadOnly,  meta = (DisplayName = "Clutch Pedal Current Value", AllowPrivateAccess = "true"))
	//clutch pedal value ranges between 0 and 100 by default
	float currentClutchPedalValue = 0;
	UPROPERTY(BlueprintReadOnly,  meta = (DisplayName = "Steering Wheel Current Value", AllowPrivateAccess = "true"))
	//steering wheel value ranges between 0 and 100 by default, with 50 in middle, 0 for complete left turn and 100 for complete right turn
	float currentSteeringWheelValue = 50;
	UPROPERTY(BlueprintReadOnly,  meta = (DisplayName = "Is Handbreak Active?", AllowPrivateAccess = "true"))
	//handbreak value is true for active handbreak and false for inactive handbreak
	bool currentHandbreakValue = false;
	UPROPERTY(BlueprintReadOnly,  meta = (DisplayName = "Is Break Assist Active?", AllowPrivateAccess = "true"))
	//Break assist value is true for active break assist and false for inactive break assist
	bool currentBreakAssistValue = false;
	UPROPERTY(BlueprintReadOnly,  meta = (DisplayName = "Fuel Handle Current Value", AllowPrivateAccess = "true"))
	//Fuel handle value ranges between 0 and 100 by default and determines fuel intake into engine wihtout gas pedal.
	float currentFuelHandleValue = 0;
};


USTRUCT(BlueprintType)
struct DYNAMICVEHICLEMOVEMENT_API FDynamicVehicleLights
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Use Vehicle Lights"))
	bool useVehicleLights = true;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Vehicle HeadLight Left", EditCondition = "useVehicleLights"))
	ULightComponent* headLightLeft = nullptr;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Vehicle HeadLight Right", EditCondition = "useVehicleLights"))
	ULightComponent* headLightRight = nullptr;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Vehicle Fog Light Left", EditCondition = "useVehicleLights"))
	ULightComponent* fogLightLeft = nullptr;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Vehicle Fog Light Right", EditCondition = "useVehicleLights"))
	ULightComponent* fogLightRight = nullptr;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Vehicle Brake Light Left", EditCondition = "useVehicleLights"))
	ULightComponent* rearLightLeft = nullptr;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Vehicle Brake Light Right", EditCondition = "useVehicleLights"))
	ULightComponent* rearLightRight = nullptr;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Vehicle Rear Light Left", EditCondition = "useVehicleLights"))
	ULightComponent* brakeLightLeft = nullptr;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Vehicle Rear Light Right", EditCondition = "useVehicleLights"))
	ULightComponent* brakeLightRight = nullptr;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Vehicle Fog Light Left Rear", EditCondition = "useVehicleLights"))
	ULightComponent* fogLightLeftRear = nullptr;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Vehicle Fog Light Right Rear", EditCondition = "useVehicleLights"))
	ULightComponent* fogLightRightRear = nullptr;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Vehicle Turn Light Left", EditCondition = "useVehicleLights"))
	ULightComponent* turnLightLeft = nullptr;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Vehicle Turn Light Right", EditCondition = "useVehicleLights"))
	ULightComponent* turnLightLeftRear = nullptr;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Vehicle Turn Light Left Rear", EditCondition = "useVehicleLights"))
	ULightComponent* turnLightRight = nullptr;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Vehicle Turn Light Right Rear", EditCondition = "useVehicleLights"))
	ULightComponent* turnLightRightRear = nullptr;
};
//////////////////////////////////////////////////////////////////////////

class DYNAMICVEHICLEMOVEMENT_API UDynamicVehicleSimulation : public UChaosVehicleSimulation
{
public:

	UDynamicVehicleSimulation(): bOverlapHit(false)
	{
		QueryBox.Init();
	}

	virtual ~UDynamicVehicleSimulation()
	{
	}

	virtual void Init(TUniquePtr<Chaos::FSimpleWheeledVehicle>& PVehicleIn) override
	{
		UChaosVehicleSimulation::Init(PVehicleIn);

		WheelState.Init(PVehicle->Wheels.Num());

		dataImpactingSimulation = FDynamicSimulationData();
	}

	virtual void UpdateConstraintHandles(TArray<FPhysicsConstraintHandle>& ConstraintHandlesIn) override;

	virtual void TickVehicle(UWorld* WorldIn, float DeltaTime, const FChaosVehicleAsyncInput& InputData, FChaosVehicleAsyncOutput& OutputData, Chaos::FRigidBodyHandle_Internal* Handle) override;

	/** Advance the vehicle simulation */
	virtual void UpdateSimulation(float DeltaTime, const FChaosVehicleAsyncInput& InputData, Chaos::FRigidBodyHandle_Internal* Handle) override;

	/** Update the vehicle state */
	virtual void UpdateState(float DeltaTime, const FChaosVehicleAsyncInput& InputData, Chaos::FRigidBodyHandle_Internal* Handle) override;

	virtual void FillOutputState(FChaosVehicleAsyncOutput& Output) override;

	/** Are enough vehicle systems specified such that physics vehicle simulation is possible */
	virtual bool CanSimulate() const override;

	/** Pass control Input to the vehicle systems */
	virtual void ApplyInput(const FControlInputs& ControlInputs, float DeltaTime) override;

	/** Perform suspension ray/shape traces */
	virtual void PerformSuspensionTraces(const TArray<Chaos::FSuspensionTrace>& SuspensionTrace, FCollisionQueryParams& TraceParams, FCollisionResponseContainer& CollisionResponse, TArray<FWheelTraceParams>& WheelTraceParams);


	/** Update the engine/transmission simulation */
	virtual void ProcessMechanicalSimulation(float DeltaTime);

	/** Process steering mechanism */
	virtual void ProcessSteering(const FControlInputs& ControlInputs);

	/** calculate and apply lateral and longitudinal friction forces from wheels */
	virtual void ApplyWheelFrictionForces(float DeltaTime);

	/** calculate and apply chassis suspension forces */
	virtual void ApplySuspensionForces(float DeltaTime, TArray<FWheelTraceParams>& WheelTraceParams);

	bool IsWheelSpinning() const;
	bool ContainsTraces(const FBox& Box, const TArray<struct Chaos::FSuspensionTrace>& SuspensionTrace);

	/** Draw 3D debug lines and things along side the 3D model */
	virtual void DrawDebug3D() override;

	float GetRelativeEngineRPM_FromSpeed(float currentSpeed = 0);

	FDynamicWheelState WheelState;	/** Cached state that holds wheel data for this frame */

	TArray<FPhysicsConstraintHandle> ConstraintHandles;

	FDynamicSimulationData dataImpactingSimulation;

	// cache trace overlap query
	TArray<FOverlapResult> OverlapResults;
	bool bOverlapHit;
	FBox QueryBox;

private:
	float targetRPM_BasedOnFuel = 0;
};

/**
 * 
 */
UCLASS(ClassGroup = (Physics), meta = (BlueprintSpawnableComponent), hidecategories = (PlanarMovement, "Components|Movement|Planar", Activation, "Components|Activation"))
class DYNAMICVEHICLEMOVEMENT_API UDynamicVehicleMovementComponent : public UChaosVehicleMovementComponent
{
	GENERATED_BODY()

public:
	UDynamicVehicleMovementComponent(const FObjectInitializer& ObjectInitializer);

	// Called every frame
	void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction);


	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement|Wheels")
	bool bSuspensionEnabled;

	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement|Wheels")
	bool bWheelFrictionEnabled;

	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement|Wheels")
	bool bLegacyWheelFrictionPosition;

	/** Wheels to create */
	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement|Wheels")
	TArray<FDynamicWheelSetup> WheelSetupsForDifferentialSystem1;

	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement|Wheels")
	TArray<FDynamicWheelSetup> WheelSetupsForDifferentialSystem2;

	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement|Wheels")
	struct FCollisionResponseContainer WheelTraceCollisionResponses;

	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement")
	bool bMechanicalSimEnabled;

	/** Engine */
	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement", meta = (EditCondition = "bMechanicalSimEnabled"))
	FDynamicVehicleEngineConfig EngineSetup;

	/** Differential */
	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement", meta = (EditCondition = "bMechanicalSimEnabled"))
	FDynamicVehicleDifferentialConfig DifferentialSetup;

	/** Transmission data */
	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement", meta = (EditCondition = "bMechanicalSimEnabled"))
	FDynamicVehicleTransmissionConfig TransmissionSetup;

	/** Transmission data */
	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement")
	FDynamicVehicleSteeringConfig SteeringSetup;

	// Our instanced wheels
	UPROPERTY(transient, duplicatetransient, BlueprintReadOnly, Category = Vehicle)
	TArray<TObjectPtr<class UDynamicWheel>> Wheels;

	/** Get current engine's rotation speed */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	float GetEngineRotationSpeed() const;

	/** Get current engine's max rotation speed */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	float GetEngineMaxRotationSpeed() const;

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	int GetNumWheels() const
	{
		return WheelStatus.Num();
	}

	UFUNCTION(BlueprintPure, Category = "Vehicles")
	static void BreakWheelStatus(const struct FDynamicWheelStatus& Status, bool& bInContact, FVector& ContactPoint, UPhysicalMaterial*& PhysMaterial
		, float& NormalizedSuspensionLength, float& SpringForce, float& SlipAngle, bool& bIsSlipping, float& SlipMagnitude, bool& bIsSkidding, float& SkidMagnitude, FVector& SkidNormal, float& DriveTorque, float& BrakeTorque, bool& bABSActivated);

	UFUNCTION(BlueprintPure, Category = "Vehicles")
	static FDynamicWheelStatus MakeWheelStatus(bool bInContact, FVector& ContactPoint, UPhysicalMaterial* PhysMaterial
		, float NormalizedSuspensionLength, float SpringForce, float SlipAngle, bool bIsSlipping, float SlipMagnitude, bool bIsSkidding, float SkidMagnitude, FVector& SkidNormal, float DriveTorque, float BrakeTorque, bool bABSActivated);

	UFUNCTION(BlueprintPure, Category = "Vehicles")
	static void BreakWheeledSnapshot(const struct FWheeledSnaphotData& Snapshot, FTransform& Transform, FVector& LinearVelocity
		, FVector& AngularVelocity, int& SelectedGear, float& EngineRPM, TArray<FWheelSnapshot>& WheelSnapshots);

	UFUNCTION(BlueprintPure, Category = "Vehicles")
	static FWheeledSnaphotData MakeWheeledSnapshot(FTransform Transform, FVector LinearVelocity, FVector AngularVelocity
		, int SelectedGear, float EngineRPM, const TArray<FWheelSnapshot>& WheelSnapshots);

	UFUNCTION(BlueprintPure, Category = "Vehicles")
	static void BreakWheelSnapshot(const struct FWheelSnapshot& Snapshot, float& SuspensionOffset
		, float& WheelRotationAngle, float& SteeringAngle, float& WheelRadius, float& WheelAngularVelocity);

	UFUNCTION(BlueprintPure, Category = "Vehicles")
	static FWheelSnapshot MakeWheelSnapshot(float SuspensionOffset, float WheelRotationAngle
		, float SteeringAngle, float WheelRadius, float WheelAngularVelocity);

	/** Get a wheels current simulation state */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	const FDynamicWheelStatus& GetWheelState(int WheelIndex) const
	{
		return WheelStatus[WheelIndex];
	}

	virtual float GetSuspensionOffset(int WheelIndex) override;
	UPhysicalMaterial* GetPhysMaterial(int WheelIndex);

	/** Set all channels to the specified response - for wheel raycasts */
	void SetWheelTraceAllChannels(ECollisionResponse NewResponse)
	{
		WheelTraceCollisionResponses.SetAllChannels(NewResponse);
	}

	/** Set the response of this body to the supplied settings - for wheel raycasts */
	void SetWheelTraceResponseToChannel(ECollisionChannel Channel, ECollisionResponse NewResponse)
	{
		WheelTraceCollisionResponses.SetResponse(Channel, NewResponse);
	}

	/** Get Collision ResponseToChannels container for this component **/
//	FORCEINLINE_DEBUGGABLE const FCollisionResponseContainer& GetTraceResponseToChannels() const { return WheelTraceCollisionResponses.GetResponseContainer(); }

	//////////////////////////////////////////////////////////////////////////
	// Public

	virtual void Serialize(FArchive& Ar) override;
	virtual void PostLoad() override;

	// Get output data from Physics Thread
	virtual void ParallelUpdate(float DeltaSeconds);

#if WITH_EDITOR
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;
#endif
	/** Are the configuration references configured sufficiently that the vehicle can be created */
	virtual bool CanCreateVehicle() const override;

	/** Used to create any physics engine information for this component */
	virtual void OnCreatePhysicsState() override;

	/** Used to shut down and pysics engine structure for this component */
	virtual void OnDestroyPhysicsState() override;

	/** display next debug page */
	static void NextDebugPage();

	/** display previous debug page */
	static void PrevDebugPage();

	/** Enable or completely bypass the ProcessMechanicalSimulation call */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void EnableMechanicalSim(bool InState)
	{
		bMechanicalSimEnabled = InState;
	}

	/** Enable or completely bypass the ApplySuspensionForces call */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void EnableSuspension(bool InState)
	{
		bSuspensionEnabled = InState;
	}

	/** Enable or completely bypass the ApplyWheelFrictionForces call */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void EnableWheelFriction(bool InState)
	{
		bWheelFrictionEnabled = InState;
	}

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetWheelClass(int WheelIndex, TSubclassOf<UDynamicWheel> InWheelClass);

	/** Grab a snapshot of the vehicle instance dynamic state */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	virtual FWheeledSnaphotData GetSnapshot() const;

	/** Set snapshot of vehicle instance dynamic state */
	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	virtual void SetSnapshot(const FWheeledSnaphotData& SnapshotIn);


	//////////////////////////////////////////////////////////////////////////
	// change handling via blueprint at runtime
	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetMaxEngineTorque(float Torque);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetDragCoefficient(float DragCoeff);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetDownforceCoefficient(float DownforceCoeff);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetDifferentialFrontRearSplit(float FrontRearSlpit);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetTractionControlEnabled(int WheelIndex, bool Enabled);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetABSEnabled(int WheelIndex, bool Enabled);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetAffectedByBrake(int WheelIndex, bool Enabled);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetAffectedByHandbrake(int WheelIndex, bool Enabled);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetAffectedBySteering(int WheelIndex, bool Enabled);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetAffectedByEngine(int WheelIndex, bool Enabled);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetWheelRadius(int WheelIndex, float Radius);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetWheelFrictionMultiplier(int WheelIndex, float Friction);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetWheelSlipGraphMultiplier(int WheelIndex, float Multiplier);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetWheelMaxBrakeTorque(int WheelIndex, float Torque);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetWheelHandbrakeTorque(int WheelIndex, float Torque);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetWheelMaxSteerAngle(int WheelIndex, float AngleDegrees);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetTorqueCombineMethod(ETorqueCombineMethod InCombineMethod, int32 WheelIndex);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetDriveTorque(float DriveTorque, int32 WheelIndex);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetBrakeTorque(float BrakeTorque, int32 WheelIndex);

	UFUNCTION(BlueprintCallable, Category = "Game|Components|DynamicVehicleMovement")
	void SetSuspensionParams(float Rate, float Damping, float Preload, float MaxRaise, float MaxDrop, int32 WheelIndex);

	/** */
	virtual TUniquePtr<Chaos::FSimpleWheeledVehicle> CreatePhysicsVehicle() override
	{
		// Make the Vehicle Simulation class that will be updated from the physics thread async callback
		VehicleSimulationPT = MakeUnique<UDynamicVehicleSimulation>();

		return UChaosVehicleMovementComponent::CreatePhysicsVehicle();
	}

	/** Allocate and setup the Chaos vehicle */
	virtual void SetupVehicle(TUniquePtr<Chaos::FSimpleWheeledVehicle>& PVehicle) override;

	virtual void ResetVehicleState() override;



	//CUSTOMIZED FUNCTIONALITY FUNCTIONS START HERE

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Differential System", BlueprintPure)
	//Returns the current in use Differenital System.
	bool IsUsingSystem1ForDifferential() const;

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Differential System", BlueprintPure)
	//Returns wether differential system can be changed.
	bool CanChangeDifferentialSystem();	

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Transmission System", BlueprintPure)
	//Returns wether high ratios are in use or low
	bool IsUsingHighGears() const;

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement", BlueprintPure)
	//Returns wether engine is started or not
	bool IsEngineStarted() const;

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement", BlueprintPure)
	//Returns wether vehicle is accelerating or decelerating
	bool IsVehicleCurrentlyAccelerating() const;

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement", BlueprintPure)
	//Returns current Engine State.
	EEngineState GetCurrentEngineState() const;

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement", BlueprintPure)
	//Returns wether vehicle is automatic or manual
	bool IsVehicleUsingAutomaticTransmission() const;

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Input", BlueprintPure)
	//Returns current vehicle input structure
	FDynamicInputData GetCurrentInputData() const;

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Transfer Case System", BlueprintPure)
	//Returns wether differential system can be changed.
	bool CanChangeTransferCasePosition();

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Transmission System", BlueprintPure)
	//Returns Current Gear. Should be used instead of GetCurrentGear function.
	int GetCurrentActiveGear() const;

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Transmission System", BlueprintPure)
	//Returns Current Gear's ratio. Takes into account high/low gear switch. Returns with final ratio and transfer case ratio multiplied
	float GetCurrentActiveGearRatio() const;

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Transmission System", BlueprintPure)
	//Final Gear ratio is multiplied with each individual gear's ratio. This eliminates that effect and returns true gear ratio.
	float GetCurrentActiveGearRatioWithoutFinalGearRatioAffect() const;

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement", BlueprintPure)
	//Returns speed in Kilometers Per hour
	float GetVehicleSpeedInKM_PerHour() const;

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Miscellanous", BlueprintPure)
	//Returns vehicle center of mass. Center of mass affects handling a lot. Use this to debug.
	FTransform GetCenterOfMass();

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Miscellanous", BlueprintPure)
	//Returns net fuel intake taking into account manual fuel handle and gas input. Base implementation only returns greater value of the two.
	//If input != -1, then input will be used to evaluate instead of gas input
	virtual float GetNetFuelIntake(float inputGasValue = -1.0f);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Functionalities", BlueprintPure)
	//Get structure representing active vehicle functionalities
	FDynamicFunctionalities GetActiveVehicleFunctionalities() const;

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Transfer Case System", BlueprintPure)
	//Get Transfer Case configuration
	FDyamicTransferCaseConfig GetTransferCaseConfig() const;

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Differential System") 
	//Set New Active Differential System. True to activate system 1, false for system 2. Truck needs to be in neutral and rest. Will return true if system changed. You can use failure reason for deubg.
	bool SetActiveSystemForDifferential(bool UseSystem1, FString& failureReason);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Transfer Case System")
	//Set New Ratio of Transfer Case if enabled.
	bool SetTransferCasePosition(ETransferCasePosition newPosition, FString &failureReason);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Transfer Case System")
	//Set New Ratio of Transfer Case if enabled.
	bool SetTransferCasePositionUsingNum(int newPosition, FString& failureReason);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Transmission System") 
	//Change between using High Gear Ratios or Low Gear Ratios. True for using High Gear. False for Low Gear. Guaranteed Success if enabled
	void ChangeTransmissionSystem(bool useHighGears);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Transmission System")
	/** Set the user input for gear accounting for high, low gear. It is suggested to use the wrappers GearUp, GearDown, GoNeutral instead of this one  (-1 reverse, 0 neutral, 1+ forward)*/
	bool SetNewGear(int GearNum, bool changeImmediately=true);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Transmission System")
	//Shift to next gear if available
	void GearUp();

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Transmission System")
	//Shift to previous gear if available. 
	void GearDown();

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Transmission System")
	//Shift to neutral directly.
	void GoNeutral();

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Transmission System")
	//Shift to reverse gear directly
	void PutInReverse();

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement")
	//Applies Throttle Input. Use this in Both Forward Movement and Reverse Movement since Gas Pedal works for both
	bool ApplyGas(float gasPedalValue); 

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement")
	//Use for Break Pedal Input
	bool ApplyBrakes(float breakPedalValue);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement")
	//Applies Steering input.
	bool SteerVehicle(float steeringWheelValue);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement")
	//Applies Clutch input. Useless for automatic vehicle
	bool ApplyClutch(float clutchPedalValue);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement")
	//Applies handbreak instantly or disables it.
	bool ChangeHandBrakeState(bool handBreakActive);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement")
	//Turns engine on/off. Equivalent to Ignition.
	bool SetEngineStarterValue(bool starterValue);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement")
	//Updates Engine State based on variable values. This is the main brain of the manual vehicle.
	//Called Internally wherever needed. Only call it if you think an action should update engine state, but is missing it somehow.
	void UpdateVehicleEngineState();

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement")
	//Toggles Break Assist. Used in heavy vehicles
	bool ToggleBreakAssist(bool enableBreakAssist);

	UFUNCTION(BlueprintCallable, Category = "Dynamic Vehicle Movement|Movement")
	//Use for Manual Fuel Handle Input. Call only when value changes
	bool AdjustFuelHandle(float fuelValue);

#if WITH_EDITOR
	virtual bool CanEditChange(const FProperty* InProperty) const override;
#endif
	//CUSTOMIZED FUNCTIONS END HERE

	

protected:

	//////////////////////////////////////////////////////////////////////////
	// Setup

	/** Re-Compute any runtime constants values that rely on setup data */
	virtual void ComputeConstants() override;

	/** Skeletal mesh needs some special handling in the vehicle case */
	virtual void FixupSkeletalMesh();

	/** Create and setup the Chaos vehicle */
	virtual void CreateVehicle();

	/** Instantiate and setup our wheel objects */
	virtual void CreateWheels();

	/** Release our wheel objects */
	virtual void DestroyWheels();

	/** Set up the chassis and wheel shapes */
	virtual void SetupVehicleShapes();

	/** Setup calculated suspension parameters */
	void SetupSuspension(TUniquePtr<Chaos::FSimpleWheeledVehicle>& PVehicle);

	/** Maps UDynamicWheel Axle to a wheel index */
	void RecalculateAxles();

	/** Get the local position of the wheel at rest */
	virtual FVector GetWheelRestingPosition(const FDynamicWheelSetup& WheelSetup);

	//////////////////////////////////////////////////////////////////////////
	// Update
	void FillWheelOutputState();

	/* Fill Async input state */
	virtual void Update(float DeltaTime) override;

	//////////////////////////////////////////////////////////////////////////
	// Debug

	/** Draw 2D debug text graphs on UI for the wheels, suspension and other systems */
	/*virtual void DrawDebug(UCanvas* Canvas, float& YL, float& YPos);*/

	/** Get distances between wheels - primarily a debug display helper */
	const FVector2D& GetWheelLayoutDimensions() const
	{
		return WheelTrackDimensions;
	}

private:
	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement", meta = (DisplayName = "Is Using System 1 For Differential?"))
	//True for System 1 in use, False for System 2. Default is System 2.
	bool useSystem1ForDifferential = false; 
	UPROPERTY()
	//True for High gear ratios being used. False for low gears. Default is High Gears
	bool usingHighGears = true; 
	UPROPERTY()
	//True for Engine Started. False for Engine off
	bool currentEngineStartedValue = false; 
	UPROPERTY()
	//True for accelerating, False for decelrating. May behave oddly at speed = 0.
	bool isVehicleAccelerating = false; 
	UPROPERTY()
	//Engine State values. Alters between varios states
	EEngineState currentEngineState = EEngineState::EngineOff;  
	UPROPERTY(BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Transfer Case System", meta = (DisplayName = "Current InUse Transfer Case Ratio", AllowPrivateAccess = "true"))
	//Ratio being applied by the transfer case system.
	float currentTransferCaseRatio = 1;
	UPROPERTY(BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Movement", meta = (DisplayName = "Can Engine Jump Start", AllowPrivateAccess = "true"))
	//Whether vehicle meets engine jump start requriements currently
	bool canEngineJumpStart = false;
	UPROPERTY()
	FDynamicInputData currentInputs;
	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement", meta = (DisplayName = "Vehicle has automatic transmission?", DisplayAfter = "bMechanicalSimEnabled"))
	//True = Automatic Car. False = Manual Car
	bool isVehicleAutomatic = false;
	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement", meta = (DisplayName = "Vehicle Optional Functionalities", DisplayAfter = "bMechanicalSimEnabled"))
	FDynamicFunctionalities vehicleFunctionalities;
	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement", meta = (DisplayName = "Transfer Case System", DisplayAfter = "vehicleFunctionalities"))
	FDyamicTransferCaseConfig transferCaseConfig;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement", meta = (DisplayName = "Vehicle Lights", AllowPrivateAccess = "true"))
	FDynamicVehicleLights vehicleLights;
	
	UPROPERTY(EditAnywhere, Category = "Dynamic Vehicle Movement|Defaults", meta = (DisplayName = "Edit Default Input Ranges"))
	//Allows to edit default input ranges.
	bool editDefaultRanges = false;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Defaults|Transmission System", meta = (DisplayName = "Clutch Threshold Value", AllowPrivateAccess = "true", EditCondition="editDefaultRanges&&isVehicleAutomatic==false"))
	//The threshold value at and after which engine allows gear change. 
	float clutchThresholdValue = 70; 
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Defaults|Transmission System", meta = (DisplayName = "Clutch Disengagment Value", AllowPrivateAccess = "true", EditCondition="editDefaultRanges&&isVehicleAutomatic==false"))
	//The value at and after which engine is disengaged from wheels. May cause unexpected behaviour if changed at runtime
	float clutchDisengagementValue = 95; 
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Defaults|Movement", meta = (DisplayName = "Gas Input Minimum Value", AllowPrivateAccess = "true", EditCondition="editDefaultRanges"))
	//Minimum possible value for gas pedal input
	float gasPedalMinValue = 0;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Defaults|Movement", meta = (DisplayName = "Gas Input Maximum Value", AllowPrivateAccess = "true", EditCondition="editDefaultRanges"))
	//Maximum possible value for gas pedal input
	float gasPedalMaxValue = 100;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Defaults|Movement", meta = (DisplayName = "Break Input Minimum Value", AllowPrivateAccess = "true", EditCondition="editDefaultRanges"))
	//Minimum possible value for break pedal input
	float breakPedalMinValue = 0;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Defaults|Movement", meta = (DisplayName = "Break Input Maximum Value", AllowPrivateAccess = "true", EditCondition="editDefaultRanges"))
	//Maximum possible value for break pedal input
	float breakPedalMaxValue = 100;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Defaults|Movement", meta = (DisplayName = "Steering Input Minimum Value", AllowPrivateAccess = "true", EditCondition="editDefaultRanges"))
	//Minimum possible value for steer wheel input. Minimum value = Complete Left Turn
	float steerWheelMinValue = 0;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Defaults|Movement", meta = (DisplayName = "Steering Input Maximum Value", AllowPrivateAccess = "true", EditCondition="editDefaultRanges"))
	//Maximum possible value for steer wheel input. Maximum value = Complete Right Turn. 
	float steerWheelMaxValue = 100;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Defaults|Transmission System", meta = (DisplayName = "Clutch Input Minimum Value", AllowPrivateAccess = "true", EditCondition="editDefaultRanges&&isVehicleAutomatic==false"))
	//Minimum possible value for clutch pedal input. 
	float clutchPedalMinValue = 0;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Defaults|Transmission System", meta = (DisplayName = "Clutch Input Maximum Value", AllowPrivateAccess = "true", EditCondition="editDefaultRanges&&isVehicleAutomatic==false"))
	//Maximum possible value for clutch pedal input.  
	float clutchPedalMaxValue = 100;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Defaults|Movement", meta = (DisplayName = "Minimum value of Fuel Intake to sustain Idle RPM", AllowPrivateAccess = "true", EditCondition = "editDefaultRanges&&isVehicleAutomatic==false"))
	//The threshold value at which engine can sustain idle RPM. Includes Net Intake from Gas Pedal and Manual Handle. 
	//See GetNetFuelIntake() for more info
	float fuelValueToSustainIdleRPM = 30;
	float previousEngineRPM; //caches last engine RPM values.
	float previousVehicleSpeed = 0; //caches last vehicle speed

	UDynamicVehicleSimulation* derivedPtrForSimulationClass = nullptr;

	/** Get distances between wheels - primarily a debug display helper */
	FVector2D CalculateWheelLayoutDimensions();


#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
	float CalcDialAngle(float CurrentValue, float MaxValue);
	void DrawDial(UCanvas* Canvas, FVector2D Pos, float Radius, float CurrentValue, float MaxValue);
#endif

	struct FCachedState
	{
		FCachedState() : WheelOffset(0.f), bIsValid(false)
		{ }

		float WheelOffset;
		bool bIsValid;
	};

	EDynamicDebugPages DebugPage;
	uint32 NumDrivenWheels; /** The number of wheels that have engine enabled checked */
	FVector2D WheelTrackDimensions;	// Wheelbase (X) and track (Y) dimensions
	TMap<UDynamicWheel*, TArray<int>> AxleToWheelMap;
	TArray<FPhysicsConstraintHandle> ConstraintHandles;
	TArray<FDynamicWheelStatus> WheelStatus; /** Wheel output status */
	TArray<FCachedState> CachedState;
	Chaos::FPerformanceMeasure PerformanceMeasure;
};

//////////////////////////////////////////////////////////////////////////

#pragma once

#include "CoreMinimal.h"
#include "Components/LightComponent.h"
#include "Sound/SoundBase.h"
#include "VehicleStructsAndEnums.generated.h"

class UDynamicWheel;
UENUM(BlueprintType)
enum class EActionErrorReason : uint8
{
	NoGasOrClutch,
	GearsSkipped,
	SpeedTooLowForGear,
	ChangeGearWithoutAppropriateClutch,
	ChangeToGearWithMovementInOppositeDirection,
	AttemptToStartInGearWithoutClutch,
	EngineEngagedOnNeutralTransferCase,
	DriveWithHandBreakOn,
	GasAndBrakeInputTogether
};

USTRUCT(BlueprintType)
struct DYNAMICVEHICLEMOVEMENT_API FVehicleActionErrors
{

	GENERATED_BODY() 
	//what caused the error?
	UPROPERTY(BlueprintReadWrite)
	EActionErrorReason errorReason;
	//Did error cause engine turn off? Example: Leaving clutch and accelerator while in gear
	UPROPERTY(BlueprintReadWrite)
	bool didEngineStall = false;

	bool areValuesFilled = false;

};


DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnVehicleActionError, FVehicleActionErrors, causingError);

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

UENUM(BlueprintType)
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

	UPROPERTY(EditAnywhere/*, Category = Setup*/, AdvancedDisplay, meta = (EditCondition = "bUseAutomaticGears!=true"))
	bool instantStopOnNoGasOrClutch = false;

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

	int GetForwardGearCount()
	{
		if (bUseAutomaticGears)
			return ForwardGearRatiosAutomatic.Num();
		else if (bUseHighLowRatios)
			return ForwardGearRatios.Num();
		else
			return ForwardGearRatiosSingular.Num();

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
		float retVal = 0;
		if (bUseAutomaticGears)
			retVal =  0.0f;
		else if (bUseHighLowRatios)
		{
			if (forwardRatios && (gearNum > 0 && gearNum <= ForwardGearRatios.Num()))
				retVal =  ForwardGearRatios[gearNum-1].MinimumSpeed / transferCaseModifer > ForwardGearRatios[gearNum - 1].MinimumSpeed? ForwardGearRatios[gearNum - 1].MinimumSpeed : ForwardGearRatios[gearNum - 1].MinimumSpeed / transferCaseModifer;
			else if (gearNum > 0 && gearNum <= ReverseGearRatios.Num())
				retVal =  ReverseGearRatios[gearNum - 1].MinimumSpeed / transferCaseModifer > ReverseGearRatios[gearNum - 1].MinimumSpeed? ReverseGearRatios[gearNum - 1].MinimumSpeed : ReverseGearRatios[gearNum - 1].MinimumSpeed / transferCaseModifer;
			else
				retVal =  0;
		}
		else
		{
			if (forwardRatios && (gearNum > 0 && gearNum < ForwardGearRatiosSingular.Num()))
				retVal =  ForwardGearRatiosSingular[gearNum].MinimumSpeed / transferCaseModifer;
			else if (gearNum > 0 && gearNum < ReverseGearRatiosSingular.Num())
				retVal =  ReverseGearRatiosSingular[gearNum].MinimumSpeed / transferCaseModifer;
			else
				retVal =  0;
		}
	
		return retVal;
	}

	float GetMaximumSpeedForGear(int gearNum, bool forwardRatios) const
	{
		if (bUseAutomaticGears)
			return 0.0f;
		else if (bUseHighLowRatios)
		{
			if (forwardRatios)
				return ForwardGearRatios[gearNum ].MaximumSpeed;
			else
				return ReverseGearRatios[gearNum ].MaximumSpeed;
		}
		else
		{
			if (forwardRatios)
				return ForwardGearRatiosSingular[gearNum].MaximumSpeed;
			else
				return ReverseGearRatiosSingular[gearNum].MaximumSpeed;
		}
	}

	float GetTransferCaseModifer()
	{
		return transferCaseModifer;
	}

	void SetTransferCaseModifier(float inRatio)
	{
		transferCaseModifer = inRatio;
	}
private:

	float transferCaseModifer = .917;

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
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Functionalities", meta = (DisplayName = "Vehicle Can Start in Gear?", AllowPrivateAccess = "true"))
	//Allow vehicle to start if not in neutral (but clutch pressed). Default is true
	bool vehicleCanStartWithoutNeutralGear = true;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Functionalities", meta = (DisplayName = "Vehicle Should Slide on Slopes?", AllowPrivateAccess = "true"))
	bool vehicleShouldSlideOnSlope = true;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Functionalities|Non Functional Aspects", meta = (DisplayName = "Vehicle has Non Functional Aspects?", AllowPrivateAccess = "true"))
	//Does Vehicle have specific transfer case with differing ratios?
	bool vehicleHasNonFunctionalAspects = true;

	void SetTransmissionNature(bool isAutomatic = true)
	{
		isVehicleAutomaticTransmission = !isAutomatic;
	}
private:
	UPROPERTY()
	bool isVehicleAutomaticTransmission = false;
};

USTRUCT(BlueprintType)
struct DYNAMICVEHICLEMOVEMENT_API FSkeletalMeshComponentReference :public FComponentReference
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Light Component Reference Structure")
	USkeletalMeshComponent* fetchedMeshComponent;

	UPROPERTY()
	//Cache to make sure name has not changed
	FName previousName;


	bool ExtractMeshComponent(AActor* Owner)
	{
		if (!IsValid(fetchedMeshComponent) || ComponentProperty != previousName)
		{
			previousName = ComponentProperty;
			UActorComponent* componentFetched = GetComponent(Owner);
			if (IsValid(componentFetched) && componentFetched->IsA(USkeletalMeshComponent::StaticClass()))
			{
				fetchedMeshComponent = Cast<USkeletalMeshComponent>(componentFetched);
				return true;
			}
			return false;
		}
		return false;
	}
};

UENUM(BlueprintType)
enum class EWiperModes :uint8
{
	Off = 0,
	FirstSpeed = 1,
	SecondSpeed = 2,
	IntermittentMode = 3
};

USTRUCT(BlueprintType)
struct DYNAMICVEHICLEMOVEMENT_API FDynamicNonFunctionalVehicleAspects
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Non Functional Aspects", meta = (DisplayName = "Vehicle Left Wiper Mesh", AllowPrivateAccess = "true"))
	FSkeletalMeshComponentReference leftWiper;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Non Functional Aspects", meta = (DisplayName = "Vehicle Right Wiper Mesh", AllowPrivateAccess = "true"))
	FSkeletalMeshComponentReference rightwiper;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Non Functional Aspects|Animations", meta = (DisplayName = "Wiper Movement Animation", AllowPrivateAccess = "true"))
	//Will be played on both wipers
	UAnimSequenceBase* wiperAnimation;

};

USTRUCT()
struct DYNAMICVEHICLEMOVEMENT_API FDynamicSimulationData
{
	GENERATED_BODY()

	UPROPERTY()
	bool isFilled = false;

	UPROPERTY()
	bool isEngineStarted = false;

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
	float netFuelIntakeIdleRPMRequirement = 0;

	UPROPERTY()
	float currentTransmissionRatio = 0;

	UPROPERTY()
	FDynamicVehicleEngineConfig engineData;

	UPROPERTY()
	FDynamicVehicleTransmissionConfig transmissionData;

	UPROPERTY()
	FDynamicFunctionalities vehicleFunctionalities;

	UPROPERTY()
	float vehicleCurrentSpeed = 0;

	UPROPERTY()
	bool isDownShifting = false;

	void FillData(float intake, bool isdownshift, float currentSpeed = 0, float currentTransmissionRatioInput = 1, bool isThrottle = false, bool isBreakAssist = false)
	{
		isFilled = true;
		isDownShifting = isdownshift;
		isThrottleActive = isThrottle;
		netFuelIntakeValue = FMath::Clamp(intake, netFuelIntakeMinRange, netFuelIntakeMaxRange);
		currentTransmissionRatio = currentTransmissionRatioInput;
		vehicleCurrentSpeed = currentSpeed;
		isBreakAssistActive = isBreakAssist;
		isEngineStarted = true;
	}

	FDynamicSimulationData()
	{
		isFilled = false;
		netFuelIntakeValue = 0;
	}

	FDynamicSimulationData(float intake, float minRangeIntake, float maxRangeIntake, float idleRPMReq, FDynamicVehicleTransmissionConfig transmissionConfig, FDynamicVehicleEngineConfig engineConfig, FDynamicFunctionalities functionalityConfig, bool settingRangesOnly = false)
	{
		if (!settingRangesOnly)
			isFilled = true;
		else
			isFilled = false;

		netFuelIntakeValue = intake;
		netFuelIntakeMinRange = minRangeIntake;
		netFuelIntakeMaxRange = maxRangeIntake;
		netFuelIntakeIdleRPMRequirement = idleRPMReq;
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
	UPROPERTY(BlueprintReadOnly, meta = (DisplayName = "Vehicle Wipe Current Mode", AllowPrivateAccess = "true"))
	EWiperModes currentWiperMode = EWiperModes::Off;
	UPROPERTY(BlueprintReadOnly, meta = (DisplayName = "Vehicle HeadLights active?", AllowPrivateAccess = "true"))
	bool vehicleHeadLightsOn = false;
	UPROPERTY(BlueprintReadOnly, meta = (DisplayName = "Vehicle Foglights active?", AllowPrivateAccess = "true"))
	bool vehicleFrontFogLightsOn = false;
	UPROPERTY(BlueprintReadOnly, meta = (DisplayName = "Vehicle Foglights active?", AllowPrivateAccess = "true"))
	bool vehicleRearFogLightsOn = false;
	UPROPERTY(BlueprintReadOnly, meta = (DisplayName = "Vehicle Parking Lights active?", AllowPrivateAccess = "true"))
	bool vehicleParkingLightsOn = false;
	UPROPERTY(BlueprintReadOnly, meta = (DisplayName = "Vehicle HeadLights Highbeam active?", AllowPrivateAccess = "true"))
	bool vehicleHeadLightsHighBeamOn = false;
	UPROPERTY(BlueprintReadOnly, meta = (DisplayName = "Vehicle Right Turn Indicators active?", AllowPrivateAccess = "true"))
	bool vehicleRightTurnIndicatorsOn = false;
	UPROPERTY(BlueprintReadOnly, meta = (DisplayName = "Vehicle Left Turn Indicators active?", AllowPrivateAccess = "true"))
	bool vehicleLeftTurnIndicatorsOn = false;
	UPROPERTY(BlueprintReadOnly, meta = (DisplayName = "Vehicle Low Horn Pressed?", AllowPrivateAccess = "true"))
	bool vehicleLowHornPressed = false;
	UPROPERTY(BlueprintReadOnly, meta = (DisplayName = "Vehicle High Horn Pressed?", AllowPrivateAccess = "true"))
	bool vehicleHighHornPressed = false;


};

USTRUCT(BlueprintType)
struct DYNAMICVEHICLEMOVEMENT_API FLightComponentReference :public FComponentReference
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Light Component Reference Structure")
	ULightComponent* fetchedLightComponent;

	UPROPERTY()
	//Cache to make sure name has not changed
	FName previousName;


	bool ExtractLightComponent(AActor *Owner)
	{
		if (!IsValid(fetchedLightComponent) || ComponentProperty!=previousName)
		{
			previousName = ComponentProperty;
			UActorComponent* componentFetched = GetComponent(Owner);
			if (IsValid(componentFetched) && componentFetched->IsA(ULightComponent::StaticClass()))
			{
				fetchedLightComponent = Cast<ULightComponent>(componentFetched);
				return true;
			}
			return false;
		}
		return false;
	}
};

USTRUCT(BlueprintType)
struct DYNAMICVEHICLEMOVEMENT_API FDynamicVehicleLights
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Use Vehicle Lights"))
	bool useVehicleLights = true;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Vehicle HeadLight Left", EditCondition = "useVehicleLights", UseComponentPicker))
	// Reference to LightComponent using Component Name. 
 	//On Instances, component picker will allow you to pick components from heirarchy 
 	FLightComponentReference headLightLeft ;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Vehicle HeadLight Right", EditCondition = "useVehicleLights", UseComponentPicker))
	// Reference to LightComponent using Component Name. 
 	//On Instances, component picker will allow you to pick components from heirarchy 
 	FLightComponentReference headLightRight ;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Vehicle Fog Light Left", EditCondition = "useVehicleLights", UseComponentPicker))
	// Reference to LightComponent using Component Name. 
 	//On Instances, component picker will allow you to pick components from heirarchy 
 	FLightComponentReference fogLightLeft ;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Vehicle Fog Light Right", EditCondition = "useVehicleLights", UseComponentPicker))
	// Reference to LightComponent using Component Name. 
 	 //On Instances, component picker will allow you to pick components from heirarchy 
 	FLightComponentReference fogLightRight ;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Vehicle Rear Light Left", EditCondition = "useVehicleLights", UseComponentPicker))
	// Reference to LightComponent using Component Name. 
 	//On Instances, component picker will allow you to pick components from heirarchy 
 	FLightComponentReference rearLightLeft ;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Vehicle Rear Light Right", EditCondition = "useVehicleLights", UseComponentPicker))
	// Reference to LightComponent using Component Name. 
 	//On Instances, component picker will allow you to pick components from heirarchy 
 	FLightComponentReference rearLightRight ;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Vehicle Brake Light Left", EditCondition = "useVehicleLights", UseComponentPicker))
	// Reference to LightComponent using Component Name. 
 	//On Instances, component picker will allow you to pick components from heirarchy 
 	FLightComponentReference brakeLightLeft ;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Vehicle Brake Light Right", EditCondition = "useVehicleLights", UseComponentPicker))
	// Reference to LightComponent using Component Name. 
 	//On Instances, component picker will allow you to pick components from heirarchy 
 	FLightComponentReference brakeLightRight ;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Vehicle Fog Light Left Rear", EditCondition = "useVehicleLights", UseComponentPicker))
	// Reference to LightComponent using Component Name. 
 	//On Instances, component picker will allow you to pick components from heirarchy 
	FLightComponentReference fogLightLeftRear ;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Vehicle Fog Light Right Rear", EditCondition = "useVehicleLights", UseComponentPicker))
	// Reference to LightComponent using Component Name. 
 	//On Instances, component picker will allow you to pick components from heirarchy 
 	FLightComponentReference fogLightRightRear ;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Vehicle Turn Light Left", EditCondition = "useVehicleLights", UseComponentPicker))
	// Reference to LightComponent using Component Name. 
 	//On Instances, component picker will allow you to pick components from heirarchy 
 	FLightComponentReference turnLightLeft ;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Vehicle Turn Light Left Rear", EditCondition = "useVehicleLights", UseComponentPicker))
	// Reference to LightComponent using Component Name. 
 	//On Instances, component picker will allow you to pick components from heirarchy 
 	FLightComponentReference turnLightLeftRear ;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Vehicle Turn Light Right", EditCondition = "useVehicleLights", UseComponentPicker))
	// Reference to LightComponent using Component Name. 
 	//On Instances, component picker will allow you to pick components from heirarchy 
 	FLightComponentReference turnLightRight ;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Dynamic Vehicle Movement|Lights", meta = (DisplayName = "Vehicle Turn Light Right Rear", EditCondition = "useVehicleLights", UseComponentPicker))
	// Reference to LightComponent using Component Name. 
 	//On Instances, component picker will allow you to pick components from heirarchy 
 	FLightComponentReference turnLightRightRear ;

	void ExtractAll(AActor *Owner)
	{
		if (useVehicleLights)
		{
			headLightLeft.ExtractLightComponent(Owner);
			headLightRight.ExtractLightComponent(Owner);
			fogLightLeft.ExtractLightComponent(Owner);
			fogLightRight.ExtractLightComponent(Owner);
			rearLightLeft.ExtractLightComponent(Owner);
			rearLightRight.ExtractLightComponent(Owner);
			brakeLightLeft.ExtractLightComponent(Owner);
			brakeLightRight.ExtractLightComponent(Owner);
			fogLightLeftRear.ExtractLightComponent(Owner);
			fogLightRightRear.ExtractLightComponent(Owner);
			turnLightLeft.ExtractLightComponent(Owner);
			turnLightLeftRear.ExtractLightComponent(Owner);
			turnLightRight.ExtractLightComponent(Owner);
			turnLightRightRear.ExtractLightComponent(Owner);
		}
	}
};

USTRUCT(BlueprintType)
struct DYNAMICVEHICLEMOVEMENT_API FDynamicVehicleSounds
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dynamic Vehicle Movement|Sounds")
	bool useEngineSounds = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dynamic Vehicle Movement|Sounds", meta = (EditCondition = "useEngineSounds"))
	bool useSeparateSoundsPerGear = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dynamic Vehicle Movement|Sounds", meta = (EditCondition = "useEngineSounds"))
	USoundBase* engineStartupSound;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dynamic Vehicle Movement|Sounds", meta = (EditCondition = "useEngineSounds"))
	USoundBase* engineStopSound;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dynamic Vehicle Movement|Sounds" , meta = (EditCondition = "useSeparateSoundsPerGear==true && useEngineSounds"))
	TArray<USoundBase*> engineGearWiseSound;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dynamic Vehicle Movement|Sounds", meta = (EditCondition = "useSeparateSoundsPerGear==false && useEngineSounds"))
	USoundBase* engineEngagedSound;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dynamic Vehicle Movement|Sounds", meta = (EditCondition = "useEngineSounds"))
	USoundBase* engineIdleSound;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dynamic Vehicle Movement|Sounds")
	USoundBase* blinkerOnSound;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dynamic Vehicle Movement|Sounds")
	USoundBase* blinkrOffSound;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dynamic Vehicle Movement|Sounds")
	USoundBase* hornSound;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dynamic Vehicle Movement|Sounds")
	USoundBase* wiperOnSound;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dynamic Vehicle Movement|Sounds")
	USoundBase* wiperOffSound;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dynamic Vehicle Movement|Sounds")
	USoundBase* wiperLoopSound;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Dynamic Vehicle Movement|Sounds")
	USoundBase* brakeSound;

};

UENUM(BlueprintType)
enum class ESlideDirection:uint8
{
	NotSliding = 0,
	SlidingForward = 1,
	SlidingBackward = 2
};

UENUM(BlueprintType)
enum class EVehicleMovementDirection : uint8
{
	None,
	Forward,
	Backward
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnVehicleStateChanged, EEngineState, newState, EEngineState, oldState);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnVehicleInputChanged, float, inputValue);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnVehicleInputChangedBool, bool, inputValue);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(FOnVehicleGearChanged, int, oldGear, int, newGear, bool, gearSkipped);
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnVehicleInputReleased);
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnVehicleStarted);
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnVehicleStopped);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnVehicleTransmissionSystemChanged, bool, usingHighGears);

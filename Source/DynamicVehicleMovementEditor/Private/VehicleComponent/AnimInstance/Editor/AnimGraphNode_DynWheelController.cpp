// Fill out your copyright notice in the Description page of Project Settings.


#include "VehicleComponent/AnimInstance/Editor/AnimGraphNode_DynWheelController.h"
#include "Kismet2/BlueprintEditorUtils.h"
#include "Kismet2/CompilerResultsLog.h"
#include "DynamicVehicleMovement/Public/VehicleComponent/AnimInstance/DynamicVehicleAnimationInstance.h"
/////////////////////////////////////////////////////
// UAnimGraphNode_DynWheelController

#define LOCTEXT_NAMESPACE "A3Nodes"


FText UAnimGraphNode_DynWheelController::GetControllerDescription() const
{
	return LOCTEXT("AnimGraphNode_WheelController", "Wheel Controller for DynamicWheeledVehicle");
}

FText UAnimGraphNode_DynWheelController::GetTooltipText() const
{
	return LOCTEXT("AnimGraphNode_WheelController_Tooltip", "This alters the wheel transform based on set up in Wheeled Vehicle. This only works when the owner is WheeledVehicle.");
}

FText UAnimGraphNode_DynWheelController::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
	FText NodeTitle;
	if (TitleType == ENodeTitleType::ListView || TitleType == ENodeTitleType::MenuTitle)
	{
		NodeTitle = GetControllerDescription();
	}
	else
	{
		// we don't have any run-time information, so it's limited to print  
		// anymore than what it is it would be nice to print more data such as 
		// name of bones for wheels, but it's not available in Persona
		NodeTitle = FText(LOCTEXT("AnimGraphNode_WheelController_Title", "Dynamic Wheel Controller"));
	}
	return NodeTitle;
}

void UAnimGraphNode_DynWheelController::ValidateAnimNodePostCompile(class FCompilerResultsLog& MessageLog, class UAnimBlueprintGeneratedClass* CompiledClass, int32 CompiledNodeIndex)
{
	// we only support vehicle anim instance
	if (CompiledClass->IsChildOf(UDynamicVehicleAnimationInstance::StaticClass()) == false)
	{
		MessageLog.Error(TEXT("@@ is only allowwed in DynamicVehicleAnimationInstance. If this is for vehicle, please change parent to be DynamicVehicleAnimationInstance (Reparent Class)."), this);
	}
}

bool UAnimGraphNode_DynWheelController::IsCompatibleWithGraph(const UEdGraph* TargetGraph) const
{
	UBlueprint* Blueprint = FBlueprintEditorUtils::FindBlueprintForGraph(TargetGraph);
	return (Blueprint != nullptr) && Blueprint->ParentClass->IsChildOf<UDynamicVehicleAnimationInstance>() && Super::IsCompatibleWithGraph(TargetGraph);
}

#undef LOCTEXT_NAMESPACE


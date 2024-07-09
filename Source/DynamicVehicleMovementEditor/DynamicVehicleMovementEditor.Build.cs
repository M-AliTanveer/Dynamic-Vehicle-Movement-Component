using System.IO;
using UnrealBuildTool;

public class DynamicVehicleMovementEditor : ModuleRules
{
    public DynamicVehicleMovementEditor(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicIncludePaths.AddRange(
            new string[] {
                Path.Combine(ModuleDirectory, "Public"),
            }
        );

        PrivateIncludePaths.AddRange(
            new string[] {
            }
        );

        PublicDependencyModuleNames.AddRange(
            new string[]
            {
                "Core",
                "PhysicsCore", // Add PhysicsCore for physics-related functionalities
                "Chaos", // Add Chaos for physics and chaos-related functionalities
                "ChaosVehicles", // Add ChaosVehicles for vehicle-related functionalities
                "DynamicVehicleMovement",

            }
        );

        PrivateDependencyModuleNames.AddRange(
            new string[]
            {
                "CoreUObject",
                "Engine",
                "Slate",
                "SlateCore",
                "ChaosVehiclesCore", // Add ChaosVehiclesCore as private dependency
                "DynamicVehicleMovement",
            }
        );

        if (Target.bBuildEditor)
        {
            PublicDependencyModuleNames.AddRange(
                new string[]
                {
                    "AnimGraph", // For anim graph related functionalities
                    "BlueprintGraph", // For blueprint graph related functionalities
                }
            );

            PrivateDependencyModuleNames.AddRange(
                new string[]
                {
                    "UnrealEd", // Add UnrealEd for editor functionalities
                    "ChaosVehiclesEditor" // Add ChaosVehiclesEditor as private dependency
                }
            );
        }

        DynamicallyLoadedModuleNames.AddRange(
            new string[]
            {
            }
        );
    }
}

using System.IO;
using UnrealBuildTool;

public class DynamicVehicleMovement : ModuleRules
{
    public DynamicVehicleMovement(ReadOnlyTargetRules Target) : base(Target)
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
                "AnimGraph", // For anim graph related functionalities
                "BlueprintGraph", // For blueprint graph related functionalities
            }
        );

        PrivateDependencyModuleNames.AddRange(
            new string[]
            {
                "CoreUObject",
                "Engine",
                "Slate",
                "SlateCore",
                "UnrealEd", // Add UnrealEd for editor functionalities
                "ChaosVehiclesCore", // Add ChaosVehiclesCore as private dependency
                "ChaosVehiclesEditor" // Add ChaosVehiclesEditor as private dependency
            }
        );

        DynamicallyLoadedModuleNames.AddRange(
            new string[]
            {
            }
        );
    }
}

// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class UE5NiagaraSandbox : ModuleRules
{
	public UE5NiagaraSandbox(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "HeadMountedDisplay", "EnhancedInput" });
	}
}

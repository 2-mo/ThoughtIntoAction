// Thought into Action build entry for the pinned XRoboToolkit Quest source (MIT).
using System;
using System.IO;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class QuestUsbBuild
{
    public static void Build()
    {
        const string version = "2021.3.45f2";
        const string scenePath = "Assets/Main.unity";
        if (Application.unityVersion != version)
            throw new InvalidOperationException("Expected Unity " + version + "; got " + Application.unityVersion);
        string output = Environment.GetEnvironmentVariable("TIA_QUEST_APK");
        if (string.IsNullOrEmpty(output))
            throw new InvalidOperationException("TIA_QUEST_APK is required; use build_quest_client.py build.");

        // Open the real scene, resolving its prefab references before validating wiring.
        var scene = EditorSceneManager.OpenScene(scenePath, OpenSceneMode.Single);
        int count = 0;
        foreach (var root in scene.GetRootGameObjects())
        foreach (var ui in root.GetComponentsInChildren<UIOperate>(true))
        {
            if (ui.TcpHandler == null)
                throw new InvalidOperationException("UIOperate is missing its serialized TcpHandler reference.");
            count++;
        }
        if (count != 1)
            throw new InvalidOperationException("Expected one scene UIOperate; found " + count);

        // Independent app id: installing this build never replaces the official app.
        PlayerSettings.SetApplicationIdentifier(BuildTargetGroup.Android, "com.thoughtintoaction.quest");
        PlayerSettings.productName = "Thought into Action Quest";
        PlayerSettings.bundleVersion = "1.0.1-tia.1";
        PlayerSettings.Android.bundleVersionCode = 10001;
        PlayerSettings.Android.useCustomKeystore = false;
        PlayerSettings.SetScriptingBackend(BuildTargetGroup.Android, ScriptingImplementation.IL2CPP);
        PlayerSettings.Android.targetArchitectures = AndroidArchitecture.ARM64;
        EditorUserBuildSettings.buildAppBundle = false;
        Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(output)));
        var report = BuildPipeline.BuildPlayer(new BuildPlayerOptions
        {
            scenes = new[] { scenePath },
            locationPathName = output,
            target = BuildTarget.Android,
            options = BuildOptions.None
        });
        if (report.summary.result != BuildResult.Succeeded)
            throw new InvalidOperationException("Quest APK build failed: " + report.summary.result);
        Debug.Log("Quest USB APK built: " + output);
    }
}

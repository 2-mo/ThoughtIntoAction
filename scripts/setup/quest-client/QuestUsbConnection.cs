// Thought into Action addition to XRoboToolkit-Unity-Client-Quest (MIT).
// Attached by UIOperate.Awake; uses the scene's existing, serialized TcpHandler.
using System;
using System.Net;
using System.Net.Sockets;
using UnityEngine;

[DisallowMultipleComponent]
public sealed class QuestUsbConnection : MonoBehaviour
{
    private void Start()
    {
        var ui = GetComponent<UIOperate>();
        if (ui == null || ui.TcpHandler == null)
        {
            Debug.LogError("Quest USB: scene UIOperate/TcpHandler reference is missing.");
            enabled = false;
            return;
        }

        string address = "127.0.0.1";
        bool autoConnect = true;
#if UNITY_ANDROID && !UNITY_EDITOR
        try
        {
            using (var player = new AndroidJavaClass("com.unity3d.player.UnityPlayer"))
            using (var activity = player.GetStatic<AndroidJavaObject>("currentActivity"))
            using (var intent = activity.Call<AndroidJavaObject>("getIntent"))
            {
                string requested = intent.Call<string>("getStringExtra", "quest_server_ip");
                if (!string.IsNullOrWhiteSpace(requested)) address = requested.Trim();
                autoConnect = intent.Call<bool>("getBooleanExtra", "quest_autoconnect", true);
            }
        }
        catch (Exception error)
        {
            Debug.LogWarning("Quest USB: launch options unavailable; using USB defaults. " + error.Message);
        }
#endif
        IPAddress parsed;
        if (!IPAddress.TryParse(address, out parsed) || parsed.AddressFamily != AddressFamily.InterNetwork)
        {
            Debug.LogError("Quest USB: quest_server_ip must be an IPv4 address; automatic connection skipped.");
            return;
        }

        // Prefill the existing controls; do not change tracking or controller toggles.
        if (ui.TargetIP != null) ui.TargetIP.text = "PC Service: " + address;
        if (ui.IpInputDialog != null)
        {
            var dialog = ui.IpInputDialog.GetComponent<IpInputDialog>();
            if (dialog != null && dialog.TmpInput != null) dialog.TmpInput.text = address;
        }
        if (!autoConnect) return;
        if (ui.ReconnectBtn != null) ui.ReconnectBtn.gameObject.SetActive(true);
        ui.TcpHandler.Connect(address);
        // TcpHandler owns retries. Focus changes never override an explicit Close().
    }
}

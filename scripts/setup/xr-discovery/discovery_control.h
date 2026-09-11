// Included only by tcpserverworker.cpp; does not change the SDK class ABI.
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QSettings>
#include <QStandardPaths>

extern "C" int quest_xr_discovery_protocol() { return 1; }

static bool questDiscoveryEnabled()
{
    const QString path = qEnvironmentVariable("QUEST_XR_DISCOVERY_CONFIG",
        QStandardPaths::writableLocation(QStandardPaths::GenericConfigLocation)
        + "/QuestArmTeleop/xr-discovery.ini");
    QSettings config(path, QSettings::IniFormat);
    config.sync();
    // Missing/unreadable settings must never enable discovery implicitly.
    const bool enabled = config.status() == QSettings::NoError
        && config.value("TCP/BroadcastEnabled", false).toBool();
    QSettings status(path + ".status", QSettings::IniFormat);
    status.setValue("Service/Protocol", 1);
    status.setValue("Service/Pid", QCoreApplication::applicationPid());
    status.setValue("Service/CheckedAtMs", QDateTime::currentMSecsSinceEpoch());
    status.setValue("TCP/BroadcastEnabled", enabled);
    status.sync();
    return enabled;
}

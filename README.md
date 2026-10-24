1、电脑上根据自己电脑型号要安装PC-Sevrvicev
https://github.com/XR-Robotics/XRoboToolkit-PC-Service

2、查看jaka主机IP（用于quest3上面的apk输入）
hostname -I

3、在quest3上面运行XRoboToolKit的app：
adb shell monkey -p com.xrobotoolkit.client.quest -c android.intent.category.LAUNCHER 1

4、quest3的MAC地址：
50:99:03:0a:38:78
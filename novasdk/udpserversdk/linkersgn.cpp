

#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <winsock2.h>
#include <ws2tcpip.h> 
#include <stdexcept>

#include "include/SenseGlove/Connect/SGConnect.hpp"
#include "include/SenseGlove/Core/Debugger.hpp"
#include "include/SenseGlove/Core/Library.hpp"
#include "include/SenseGlove/Core/SenseCom.hpp"

#include "include/SenseGlove/Core/HandLayer.hpp"

#include "include/SenseGlove/Core/HandPose.hpp"
#include "include/SenseGlove/Core/Quat.hpp"
#include "include/SenseGlove/Core/Vect3D.hpp"

#include "include/SenseGlove/Core/HapticGlove.hpp"
#include "include/SenseGlove/Core/StringUtils.hpp"
#include "include/SenseGlove/Core/Tracking.hpp"

#include "include/SenseGlove/Core/Tracking.hpp"
#include "include/nlohmann/json.hpp"
#include "include/HandDataJSON.hpp"


#define PORT 7000
#define BROADCAST_PORT 7000
#define BUFFER_SIZE 1024

using json = nlohmann::json;
using namespace SGCore;
using namespace SGCore::Kinematics;

#pragma comment(lib, "ws2_32.lib")


void delay(int milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}


HandDataJSON create_and_parse() {
    // 1. 创建并填充数据
    HandDataJSON data;
    /*data.right().thumb = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    data.right().thumb = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    data.right().thumb = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    data.right().thumb = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    data.right().thumb = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };*/
    // 2. 序列化为JSON字符串
    std::string jsonStr = data.to_json_string();

    // 3. 从JSON重建对象并返回
    return HandDataJSON(jsonStr);
}

static void UseVibration(bool rightHand, float amplitude, float duration, float frequency, SGCore::EHapticLocation location)
{
    if (!HandLayer::DeviceConnected(rightHand))
        return;

    std::string hand = rightHand ? "right hand" : "left hand";
    if (HandLayer::SupportsCustomWaveform(rightHand, location)) {
        SGCore::CustomWaveform waveform{ amplitude, duration, frequency };
        HandLayer::SendCustomWaveform(rightHand, waveform, location);
    }
    else {
        std::cout << ("The " + hand + " does not support Custom Waveforms (at " + HapticGlove::ToString(location) + "), so we're sending a vibration to the Index Finger instead") << std::endl;
        //whole hand and / or custom waveforms not supported. So we're pulsing the index finger instead.
        HandLayer::QueueCommand_VibroLevel(rightHand, 1, 1.0f, true);
        std::this_thread::sleep_for(std::chrono::milliseconds((int32_t)(duration * 1000))); //s to ms
        HandLayer::QueueCommand_VibroLevel(rightHand, 1, 0.0f, true);//turn it back off
    }
}

int main() {
    // 初始化，加载版本号
    std::cout << ("Nova核心包版本号: " + SGCore::Library::Version() + ", 最新版本号: " + SGCore::Library::BackendVersion());
    if (SGCore::Library::GetBackendType() == SGCore::EBackendType::SharedMemory) // By default, your library will be compiled to use Shared Memory via the SGConnect library
    {
        std::cout << (" using " + SGCore::Library::SGConnectVersion()); // If you replace SGConnect.dll, this will give you its current version number.
    }
    std::cout << std::endl;
    std::cout << ("The source code for this program is located in the SGCoreCs/test/ folder.") << std::endl;
    std::cout << ("=========================================================================") << std::endl;

    // 连接SenseCom软件
    {
        bool connectionsActive = SGCore::SenseCom::ScanningActive(); // returns true if SenseCom (or another program) has started the SenseGlove Communications Process.
        if (!connectionsActive)                                      // If this process is not running yet, we can "Force-Start" SenseCom. Provided it has run on this PC at least once.
        {
            std::cout << ("SenseCom is not yet running. Without it, we cannot connect to SenseGlove devices.") << std::endl;
            bool startedSenseCom = SGCore::SenseCom::StartupSenseCom(); // Returns true if the process was started.
            if (startedSenseCom)
            {
                std::cout << ("Successfully started SenseCom. It will take a few seconds to connect...") << std::endl;
                connectionsActive = SGCore::SenseCom::ScanningActive(); // this will return false immedeately after you called StartupSenseCom(). Because the program has yet to initialize.
                //  Even if SenseCom started and the connections process is active, there's no guarantee that the user has turned their device(s) on. More on that later.
            }
            else // If StartupSenseCom() returns false, you've either never run SenseCom, or it is already running. But at that point, the ScanningActive() should have returned true.
            {
                std::cout << ("Could not Start the SenseCom process.") << std::endl;
            }
            std::cout << ("-------------------------------------------------------------------------") << std::endl;
        }
    }

    // 连接设备
    {
        int32_t gloveAmount = HandLayer::GlovesConnected(); // GlovesConnected gives you the amount of gloves connected to your system.
        while (gloveAmount == 0)                            // For this exercise, I'll keep trying to connect to a glove.
        {
            std::cout << ("Failed to Detect a Haptic Glove on your System. Please ensure your device is paired or connected via USB.") << std::endl;
            std::cout << ("Press Return to try again...") << std::endl;
            system("pause");
            gloveAmount = HandLayer::GlovesConnected();
        }

        // When we get here, we've got one glove connected. Yay.

        if (gloveAmount == 1)
        {
            std::cout << ("There is 1 Haptic Glove connected to your system.") << std::endl;
            bool rightHand = HandLayer::GetFirstGloveHandedness();
            std::cout << "It is a ";
            std::cout << (rightHand ? "Right" : "Left");
            std::cout << " handed glove of type " + SGDevice::ToString(HandLayer::GetDeviceType(rightHand)) << std::endl;
        }
        else
        {
            std::cout << ("There are " + std::to_string(gloveAmount) + " Haptic Gloves connected to your system.") << std::endl;

            if (HandLayer::DeviceConnected(true))
            {
                std::cout << ("The right hand is a " + SGDevice::ToString(HandLayer::GetDeviceType(true))) << std::endl;
            }
            else
            {
                std::cout << ("There is no right hand connected.") << std::endl;
            }

            if (HandLayer::DeviceConnected(false))
            {
                std::cout << ("The left hand is a " + SGDevice::ToString(HandLayer::GetDeviceType(false))) << std::endl;
            }
            else
            {
                std::cout << ("There is no left hand connected.") << std::endl;
            }
        }
        std::cout << ("-------------------------------------------------------------------------") << std::endl;
    }

    // 初始化解析后的全局数据
    HandDataJSON globalData = create_and_parse();
   
    //std::cout << "按回车键继续...";
    //std::cin.get();

    // 初始化Winsock
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        std::cerr << "WSAStartup failed: " << WSAGetLastError() << std::endl;
        return 1;
    }

    // 创建UDP socket
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) {
        std::cerr << "Socket creation failed: " << WSAGetLastError() << std::endl;
        WSACleanup();
        return 1;
    }

    // 设置socket为非阻塞模式
    u_long mode = 1; // 1=非阻塞, 0=阻塞
    if (ioctlsocket(sock, FIONBIO, &mode) == SOCKET_ERROR) {
        std::cerr << "ioctlsocket failed: " << WSAGetLastError() << std::endl;
        closesocket(sock);
        WSACleanup();
        return 1;
    }

    // 允许广播
    int broadcast = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (char*)&broadcast, sizeof(broadcast)) == SOCKET_ERROR) {
        std::cerr << "setsockopt(SO_BROADCAST) failed: " << WSAGetLastError() << std::endl;
        closesocket(sock);
        WSACleanup();
        return 1;
    }

    // 绑定接收端口
    sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(PORT);
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    if (bind(sock, (sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
        std::cerr << "Bind failed: " << WSAGetLastError() << std::endl;
        closesocket(sock);
        WSACleanup();
        return 1;
    }

    std::cout << "UDP server running on port " << PORT << std::endl;



    //sockaddr_in broadcastAddr{};
    //broadcastAddr.sin_family = AF_INET;
    //broadcastAddr.sin_port = htons(BROADCAST_PORT);
    //if (inet_pton(AF_INET, "192.168.95.1", &broadcastAddr.sin_addr) <= 0) {
    //    std::cerr << "inet_pton failed for broadcast address" << std::endl;
    //    closesocket(sock);
    //    WSACleanup();
    //    return 1;
    //}


    while (true) {
        char buffer[BUFFER_SIZE];
        sockaddr_in clientAddr{};
        int clientAddrLen = sizeof(clientAddr);

        int bytesReceived = recvfrom(sock, buffer, BUFFER_SIZE, 0,
            (sockaddr*)&clientAddr, &clientAddrLen);

        if (bytesReceived > 0) {
            buffer[bytesReceived] = '\0';

            char clientIP[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &clientAddr.sin_addr, clientIP, INET_ADDRSTRLEN);

            try {
                json response = json::parse(buffer);
                SensorData data = globalData.from_json(response);

                // 打印解析结果
                //std::cout << "时间戳: " << data.timestamp << "\n";
                //std::cout << "数据类型: " << data.datatype << "\n";

                //std::cout << "右手拇指力传感器: "
                //    << data.right.index.normalforce << ", "
                //    << data.right.index.approachforce << "\n" << std::endl;


                std::vector<float> rightfingerFfb = { (float)data.right.thumb.normalforce / 255.0f, (float)data.right.index.normalforce / 255.0f, (float)data.right.middle.normalforce / 255.0f, (float)data.right.ring.normalforce / 255.0f, 0.0f };//there's 5 fingers.
                HandLayer::QueueCommand_ForceFeedbackLevels(true, rightfingerFfb, true);
                std::vector<float> leftfingerFfb = { (float)data.left.thumb.normalforce / 255.0f, (float)data.left.index.normalforce / 255.0f, (float)data.left.middle.normalforce / 255.0f, (float)data.left.ring.normalforce / 255.0f, 0.0f };//there's 5 fingers.
                HandLayer::QueueCommand_ForceFeedbackLevels(false, leftfingerFfb, true);


                
                //float duration = 0.200f;//200 ms
                //float frequency = 80.0f;//80 Hz
                //std::vector<float> tamplitude_r = { (float)data.right.thumb.approachforce / 255.0f, (float)data.right.index.approachforce / 255.0f, (float)data.right.middle.approachforce / 255.0f, (float)data.right.ring.approachforce / 255.0f, 0.0f };

                //EHapticLocation location0 = EHapticLocation::ThumbTip;
                //EHapticLocation location1 = EHapticLocation::IndexTip;
                //EHapticLocation location2 = EHapticLocation::MiddleTip;
                //EHapticLocation location3 = EHapticLocation::RingTip;
                //UseVibration(true, tamplitude_r[0], duration, frequency, location0);              
                //UseVibration(true, tamplitude_r[1], duration, frequency, location1);               
                //UseVibration(true, tamplitude_r[2], duration, frequency, location2);            
                //UseVibration(true, tamplitude_r[3], duration, frequency, location3);

                //std::vector<float> tamplitude_l = { (float)data.left.thumb.approachforce / 255.0f, (float)data.left.index.approachforce / 255.0f, (float)data.left.middle.approachforce / 255.0f, (float)data.left.ring.approachforce / 255.0f, 0.0f };
                //UseVibration(true, tamplitude_l[0], duration, frequency, location0);
                //UseVibration(true, tamplitude_l[1], duration, frequency, location1);
                //UseVibration(true, tamplitude_l[2], duration, frequency, location2);
                //UseVibration(true, tamplitude_l[3], duration, frequency, location3);
            }
            catch (const json::parse_error& e) {
                std::cerr << "JSON parse error: " << e.what() << std::endl;
            }
            try {
                HandPose righthandPose;
                HandPose lefthandPose;
                std::vector< std::vector<Kinematics::Vect3D> > righthandAngles;
                std::vector< std::vector<Kinematics::Vect3D> > lefthandAngles;
                if (HandLayer::GetHandPose(true, righthandPose)) {
                    righthandAngles = righthandPose.GetHandAngles();
                    globalData.right().thumb = { righthandAngles[0][0].GetX(),righthandAngles[0][0].GetY(), righthandAngles[0][0].GetZ(), righthandAngles[0][1].GetY(), righthandAngles[0][2].GetY(), 0 };
                    globalData.right().index = { 0,righthandAngles[1][0].GetY(), righthandAngles[1][0].GetZ(), 0, righthandAngles[1][1].GetY(), righthandAngles[1][2].GetY() };
                    globalData.right().middle = { 0,righthandAngles[2][0].GetY(), righthandAngles[2][0].GetZ(),0, righthandAngles[2][1].GetY(), righthandAngles[2][2].GetY() };
                    globalData.right().ring = { 0,righthandAngles[3][0].GetY(), righthandAngles[3][0].GetZ(), 0, righthandAngles[3][1].GetY(), righthandAngles[3][2].GetY() };
                    globalData.right().pinky = { 0,righthandAngles[4][0].GetY(), righthandAngles[4][0].GetZ(),0, righthandAngles[4][1].GetY(), righthandAngles[4][2].GetY() };
                    //std::cout << righthandAngles[0][0].GetX() << ", " << righthandAngles[0][0].GetY() << ", "<< righthandAngles[0][0].GetZ() << ", " << righthandAngles[0][1].GetY()  << ", " << righthandAngles[0][2].GetY() << "\n" << std::endl;
                }
                else {
                 }

                if (HandLayer::GetHandPose(false, lefthandPose)) {
                    lefthandAngles = lefthandPose.GetHandAngles();
                    globalData.left().thumb = { lefthandAngles[0][0].GetX(),lefthandAngles[0][0].GetY(), lefthandAngles[0][0].GetZ(), lefthandAngles[0][1].GetY(), lefthandAngles[0][2].GetY(), 0 };
                    globalData.left().index = { 0,lefthandAngles[1][0].GetY(), lefthandAngles[1][0].GetZ(), 0, lefthandAngles[1][1].GetY(), lefthandAngles[1][2].GetY() };
                    globalData.left().middle = { 0,lefthandAngles[2][0].GetY(), lefthandAngles[2][0].GetZ(),0, lefthandAngles[2][1].GetY(), lefthandAngles[2][2].GetY() };
                    globalData.left().ring = { 0,lefthandAngles[3][0].GetY(), lefthandAngles[3][0].GetZ(), 0, lefthandAngles[3][1].GetY(), lefthandAngles[3][2].GetY() };
                    globalData.left().pinky = { 0,lefthandAngles[4][0].GetY(), lefthandAngles[4][0].GetZ(),0, lefthandAngles[4][1].GetY(), lefthandAngles[4][2].GetY() };
                    //std::cout << lefthandAngles[0][0].GetX() << ", " << lefthandAngles[0][0].GetY() << ", "<< lefthandAngles[0][0].GetZ() << ", " << lefthandAngles[0][1].GetY()  << ", " << lefthandAngles[0][2].GetY() << "\n" << std::endl;
                    //std::cout << ", " << globalData.left().thumb << std::endl;
                }
                else {
                    }
                //globalData.right().thumb = { 0,1,2,3,4,5};
                //globalData.right().index = { 10,11,12,13,14,15 };
                //globalData.right().middle = { 20,21,22,23,24,25 };
                //globalData.right().ring = { 30,31,32,33,34,35 };
                //globalData.right().pinky = { 40,41,42,43,44,45 };
                std::string msgStr = globalData.to_json_string();
                //std::cout << msgStr << std::endl;
                if (sendto(sock, msgStr.c_str(), msgStr.size(), 0,
                    (sockaddr*)&clientAddr, clientAddrLen) == SOCKET_ERROR) {
                    std::cerr << "Broadcast failed: " << WSAGetLastError() << std::endl;
                }
            }
            catch (const std::invalid_argument& e) {
                std::cerr << "捕获到异常: " << e.what() << std::endl;
            }
        }
        else if (bytesReceived == SOCKET_ERROR) {
            int error = WSAGetLastError();
            if (error != WSAEWOULDBLOCK) {
                //std::cerr << "Receive error: " << error << std::endl;
            }
        }

        Sleep(10);
    }

    closesocket(sock);
    WSACleanup();
    return 0;
}
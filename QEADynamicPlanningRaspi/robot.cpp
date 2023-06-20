#include "robot.h"


struct RobotInfo robotInfo;

int robotSerial;
pthread_t robotThreadHandle;
char robotRecvBuff[1024] = {0};
int robotRecvOffset = 0;

volatile bool motionOK = false;

bool startsWith(const char* str1, const char* str2)
{
    int len1 = strlen(str1), len2 = strlen(str2);
    if (len1 < len2) return false;
    for (int i = 0; i < len2; ++i) {
        if (str1[i] != str2[i]) return false;
    }
    return true;
}

void Handle_Robot_Message(char* message, int len)
{
    message[len] = 0;

    const char* format = nullptr;
    if (startsWith(message, "[imu]")) {
        format = "[imu]heading:%lf pitch:%lf roll:%lf";
        sscanf(message, format, &robotInfo.heading, &robotInfo.pitch, &robotInfo.roll);
    } else if (startsWith(message, "[motion]OK")) {
        motionOK = true;
    }
}

void Robot_Init()
{
    robotInfo.xPos = 0;
    robotInfo.yPos = 0;
}


void* RobotThread(void*)
{
    robotSerial = serialOpen(ROBOT_DEV_PATH, ROBOT_BAUD);
    if ( robotSerial < 0) {
        std::cout << "cannot open robot dev!" << std::endl;
        exit(0);
        return nullptr;
    }
    while (true) {
        if (robotRecvOffset >= 1024) robotRecvOffset = 0;
        robotRecvBuff[robotRecvOffset] = serialGetchar(robotSerial);
        if (robotRecvBuff[robotRecvOffset] == '\n') {
            Handle_Robot_Message(robotRecvBuff, robotRecvOffset);
            robotRecvOffset = 0;
        } else {
            ++robotRecvOffset;
        }
    }
    return nullptr;
}

void Robot_Start()
{
    pthread_create(&robotThreadHandle, nullptr, RobotThread, nullptr);
}

/*
 * @brief 机器人向前移动一段距离
 * @param speed 移动速度
 * @param dis 移动距离
 */
void Robot_MoveForward(double speed, double dis)
{
    motionOK = false;
    char message[128] = {0};
    snprintf(message, 128, "[forward]speed=%.4lf dis=%.4lf\n", speed, dis);
    std::cout << message << std::endl;
    serialPuts(robotSerial, message);
    while (!motionOK) {
        usleep(1000);
    }
}

/*
 * @brief 机器人向后移动一段距离
 * @param speed 移动速度
 * @param dis 移动距离
 */
void Robot_MoveBackward(double speed, double dis)
{
    motionOK = false;
    char message[128] = {0};
    snprintf(message, 128, "[backward]speed=%.4lf dis=%.4lf\n", speed, dis);
    std::cout << message << std::endl;
    serialPuts(robotSerial, message);
    while (!motionOK) {
        usleep(1000);
    }
}

/*
 * @brief 机器人转弯
 * @param speed 移动速度
 * @param radian 要转到的弧度
 */
void Robot_SpinTo(double speed, double radian)
{
    motionOK = false;
    char message[128] = {0};
    snprintf(message, 128, "[spin]speed=%.4lf radian=%.4lf\n", speed, radian);
    std::cout << message << std::endl;
    serialPuts(robotSerial, message);
    while (!motionOK) {
        usleep(1000);
    }
}


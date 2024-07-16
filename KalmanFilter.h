#ifndef KALMANFILTER_H
#define KALMANFILTER_H

class KalmanFilter {
public:
    KalmanFilter(float Q_angle, float Q_bias, float R_measure);
    void setAngle(float angle); // 初始化角度
    float getAngle(float newAngle, float newRate, float dt);

private:
    float Q_angle; // 过程噪声协方差（角度）
    float Q_bias;  // 过程噪声协方差（偏差）
    float R_measure; // 测量噪声协方差

    float angle; // 滤波器计算得到的角度
    float bias;  // 偏差估计
    float P[2][2]; // 误差协方差矩阵
};

#endif

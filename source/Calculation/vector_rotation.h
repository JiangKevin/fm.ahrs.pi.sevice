#ifndef COORDINATE_TRANSFORM_HPP
#define COORDINATE_TRANSFORM_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <stdexcept>

namespace Coordinate
{

    /**
     * 将向量从设备坐标系旋转到ENU坐标系（使用四元数）
     * @param vector 待旋转的向量（设备坐标系）
     * @param quaternion 表示旋转的四元数
     * @return 旋转后的向量（ENU坐标系）
     */
    Eigen::Vector3f deviceToENU( const Eigen::Vector3f& vector, const Eigen::Quaternionf& quaternion )
    {

        // 四元数旋转向量: v_enu = q * v_device * q^(-1)
        return quaternion * vector;
    }

    /**
     * 将向量从设备坐标系旋转到ENU坐标系（使用欧拉角生成四元数）
     * @param vector 待旋转的向量（设备坐标系）
     * @param roll 绕X轴旋转角度（横滚，弧度）
     * @param pitch 绕Y轴旋转角度（俯仰，弧度）
     * @param yaw 绕Z轴旋转角度（航向，弧度）
     * @return 旋转后的向量（ENU坐标系）
     */
    Eigen::Vector3f deviceToENU( const Eigen::Vector3f& vector, float roll, float pitch, float yaw )
    {

        // 从欧拉角创建四元数（ZYX顺序，即Yaw-Pitch-Roll）
        Eigen::Quaternionf quaternion = Eigen::AngleAxisf( yaw, Eigen::Vector3f::UnitZ() ) * Eigen::AngleAxisf( pitch, Eigen::Vector3f::UnitY() ) * Eigen::AngleAxisf( roll, Eigen::Vector3f::UnitX() );

        return deviceToENU( vector, quaternion.normalized() );
    }

    /**
     * 计算磁偏角修正后的四元数
     * @param quaternion 原始四元数
     * @param declination 磁偏角（弧度）
     * @return 修正后的四元数
     */
    Eigen::Quaternionf applyMagneticDeclination( const Eigen::Quaternionf& quaternion, float declination )
    {

        // 创建绕Z轴（天顶）的旋转四元数
        Eigen::Quaternionf declinationQuat( Eigen::AngleAxisf( declination, Eigen::Vector3f::UnitZ() ) );

        // 组合四元数: q_new = q_declination * q_original
        return declinationQuat * quaternion;
    }

    /**
     * 将向量从设备坐标系旋转到ENU坐标系（考虑磁偏角）
     * @param vector 待旋转的向量（设备坐标系）
     * @param quaternion 表示旋转的四元数
     * @param declination 磁偏角（弧度）
     * @return 旋转后的向量（ENU坐标系）
     */
    Eigen::Vector3f deviceToENUWithDeclination( const Eigen::Vector3f& vector, const Eigen::Quaternionf& quaternion, float declination )
    {

        // 应用磁偏角修正
        Eigen::Quaternionf correctedQuat = applyMagneticDeclination( quaternion, declination );

        return deviceToENU( vector, correctedQuat );
    }

    /**
     * 将ENU坐标系下的向量转换为航向角（0-360度，正北为0度，顺时针增加）
     * @param vector ENU坐标系下的水平向量（忽略Z分量）
     * @return 航向角（弧度）
     */
    float vectorToHeading( const Eigen::Vector3f& vector )
    {
        // 计算向量在水平面（XY平面）的投影角度
        float heading = std::atan2( vector.x(), vector.y() );  // atan2(东, 北)

        // 确保角度在0-2π范围内
        if ( heading < 0 )
        {
            heading += 2 * M_PI;
        }

        return heading;
    }
    //
    int test( Eigen::Vector3f deviceMagneticField, Eigen::Quaternionf quaternion, Eigen::VectorXf& std_mag, Eigen::VectorXf& a_std_mag )
    {
        // 1. 设备坐标系下的磁场测量值（单位：μT）
        // Eigen::Vector3f deviceMagneticField( 10.0f, 20.0f, 30.0f );

        // 2. 设备姿态（四元数表示）
        // 示例：假设设备水平放置，Z轴朝上，Y轴指向北偏东45度
        // Eigen::Quaternionf quaternion( Eigen::AngleAxisf( M_PI / 4, Eigen::Vector3f::UnitZ() ) );  // 绕Z轴旋转45度

        // 3. 磁偏角（北京地区约为-6度，即-0.105弧度）
        float declination = -0.105f;

        try
        {
            // 方法1：直接使用四元数转换
            Eigen::Vector3f enuMagneticField = Coordinate::deviceToENU( deviceMagneticField, quaternion );

            std::cout << "设备坐标系磁场: [" << deviceMagneticField.x() << ", " << deviceMagneticField.y() << ", " << deviceMagneticField.z() << "] μT" << std::endl;

            std::cout << "ENU坐标系磁场（未修正磁偏角）: [" << enuMagneticField.x() << " (东), " << enuMagneticField.y() << " (北), " << enuMagneticField.z() << " (天)] μT" << std::endl;
            std_mag << enuMagneticField.x(), enuMagneticField.y(), enuMagneticField.z();

            // 方法2：考虑磁偏角修正
            Eigen::Vector3f enuMagneticFieldCorrected = Coordinate::deviceToENUWithDeclination( deviceMagneticField, quaternion, declination );

            std::cout << "ENU坐标系磁场（修正磁偏角后）: [" << enuMagneticFieldCorrected.x() << " (东), " << enuMagneticFieldCorrected.y() << " (北), " << enuMagneticFieldCorrected.z() << " (天)] μT" << std::endl;
            a_std_mag << enuMagneticFieldCorrected.x(), enuMagneticFieldCorrected.y(), enuMagneticFieldCorrected.z();
            // 计算航向角（正北为0度，顺时针增加）
            float heading = Coordinate::vectorToHeading( enuMagneticFieldCorrected ) * 180 / M_PI;
            std::cout << "航向角: " << heading << " 度" << std::endl;
        }
        catch ( const std::exception& e )
        {
            std::cerr << "Error: " << e.what() << std::endl;
            return 1;
        }
    }
}  // namespace Coordinate

#endif  // COORDINATE_TRANSFORM_HPP
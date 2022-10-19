/**
 * @file planning.h
 * @author feifei (SY2113102@buaa.edu.cn)
 * @brief 
 * @version 0.1
 * @date 2022-10-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef PLANNING_START
#define PLANNING_START

#include <string>

#include "modules/common/status/status.h"

namespace apollo {
namespace planning {
/**
 * @brief planning module入口部分
 * 
 */
class Planning
{
private:
    /* data */
public:
    Planning() = default;
    virtual ~Planning();
    /**
     * @brief module name
     * @return module name
     */
    std::string Name() const;
    /**
     * @brief 初始化
     * @return initialization status
     */
    apollo::common::Status Init();
};

Planning::Planning(/* args */)
{
}

Planning::~Planning()
{
}





} // namespace planning
} // namespace apollo
#endif // PLANNING_START
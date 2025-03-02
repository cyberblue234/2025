#pragma once

#include <frc/controller/ProfiledPIDController.h>
#include "Constants.h"

/// @todo Add pretty much anything else that could possibly exist
// Create nt tables for each individual thing (motor, controller, etc) 

template <class unit>
class ProfiledPIDControllerTelemtry
{
public:

private:
    frc::ProfiledPIDController<unit> &controller;
};


static std::shared_ptr<nt::NetworkTable> GetTable(const std::string &tableName)
{
    return nt::NetworkTableInstance::GetDefault().GetTable(tableName);
}
static nt::IntegerSubscriber GetIntegerSubscriber(const std::string &tableName, const std::string &entryName)
{
    return GetTable(tableName)->GetIntegerTopic(entryName).Subscribe(0);
}
static nt::IntegerArraySubscriber GetIntegerArraySubscriber(const std::string &tableName, const std::string &entryName)
{
    return GetTable(tableName)->GetIntegerArrayTopic(entryName).Subscribe(std::span<int64_t>{});
}
static nt::DoubleSubscriber GetDoubleSubscriber(const std::string &tableName, const std::string &entryName)
{
    return GetTable(tableName)->GetDoubleTopic(entryName).Subscribe(0.0);
}
static nt::DoubleArraySubscriber GetDoubleArraySubscriber(const std::string &tableName, const std::string &entryName)
{
    return GetTable(tableName)->GetDoubleArrayTopic(entryName).Subscribe(std::span<double>{});
}
static nt::StringSubscriber GetStringSubscriber(const std::string &tableName, const std::string &entryName)
{
    return GetTable(tableName)->GetStringTopic(entryName).Subscribe("");
}
static nt::IntegerPublisher GetIntegerPublisher(const std::string &tableName, const std::string &entryName)
{
    return GetTable(tableName)->GetIntegerTopic(entryName).Publish();
}
static nt::IntegerArrayPublisher GetIntegerArrayPublisher(const std::string &tableName, const std::string &entryName)
{
    return GetTable(tableName)->GetIntegerArrayTopic(entryName).Publish();
}
static nt::DoublePublisher GetDoublePublisher(const std::string &tableName, const std::string &entryName)
{
    return GetTable(tableName)->GetDoubleTopic(entryName).Publish();
}
static nt::DoubleArrayPublisher GetDoubleArrayPublisher(const std::string &tableName, const std::string &entryName)
{
    return GetTable(tableName)->GetDoubleArrayTopic(entryName).Publish();
}

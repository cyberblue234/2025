#pragma once

#include <frc/controller/ProfiledPIDController.h>
#include "Constants.h"

/// @todo Add pretty much anything else that could possibly exist
// Create nt tables for each individual thing (motor, controller, etc) 

// Proof of concept
// class ProfiledPIDControllerMetersTelemetry
// {
// public:
//     ProfiledPIDControllerMetersTelemetry(std::string name, frc::ProfiledPIDController<units::meter> *controller)
//     {
//         this->name = name;
//         this->controller = controller;
//         kP = GetDoubleSubscriber(name, "kP", controller->GetP());
//         kP = GetDoubleSubscriber(name, "kI", controller->GetI());
//         kP = GetDoubleSubscriber(name, "kD", controller->GetD());
//         kP = GetDoubleSubscriber(name, "Max Velocity", controller->GetConstraints().maxVelocity.value());
//         kP = GetDoubleSubscriber(name, "Max Acceleration", controller->GetConstraints().maxAcceleration.value());
//     }
//     void Update()
//     {
//         double newP = kP.Get();
//         if (newP != controller->GetP()) controller->SetP(newP);
//         double newI = kI.Get();
//         if (newI != controller->GetI()) controller->SetI(newI);
//         double newD = kD.Get();
//         if (newD != controller->GetD()) controller->SetD(newD);
        
//         double newMaxVel = maxVelocity.Get();
//         if (newMaxVel != controller->GetConstraints().maxVelocity.value()) 
//             controller->SetConstraints(frc::TrapezoidProfile<units::meter>::Constraints{units::meters_per_second_t{newMaxVel}, controller->GetConstraints().maxAcceleration});
//         double newMaxAccel = maxAcceleration.Get();
//         if (newMaxAccel != controller->GetConstraints().maxAcceleration.value()) 
//             controller->SetConstraints(frc::TrapezoidProfile<units::meter>::Constraints{controller->GetConstraints().maxVelocity, units::meters_per_second_squared_t{newMaxAccel}});
//     }
// private:
//     frc::ProfiledPIDController<units::meter> *controller;
//     std::string name;
//     nt::DoubleSubscriber kP;
//     nt::DoubleSubscriber kI;
//     nt::DoubleSubscriber kD;
//     nt::DoubleSubscriber maxVelocity;
//     nt::DoubleSubscriber maxAcceleration;
// };

inline static std::shared_ptr<nt::NetworkTable> GetTable(const std::string &tableName)
{
    return nt::NetworkTableInstance::GetDefault().GetTable(tableName);
}

// Subscribers:

inline static nt::BooleanSubscriber GetBooleanSubscriber(const std::string &tableName, const std::string &entryName, const nt::BooleanTopic::ParamType &defaultValue = {})
{
    return GetTable(tableName)->GetBooleanTopic(entryName).Subscribe(defaultValue);
}
inline static nt::BooleanArraySubscriber GetBooleanArraySubscriber(const std::string &tableName, const std::string &entryName, const nt::BooleanArrayTopic::ParamType &defaultValue = {})
{
    return GetTable(tableName)->GetBooleanArrayTopic(entryName).Subscribe(defaultValue);
}

inline static nt::IntegerSubscriber GetIntegerSubscriber(const std::string &tableName, const std::string &entryName, const nt::IntegerTopic::ParamType &defaultValue = {})
{
    return GetTable(tableName)->GetIntegerTopic(entryName).Subscribe(defaultValue);
}
inline static nt::IntegerArraySubscriber GetIntegerArraySubscriber(const std::string &tableName, const std::string &entryName, const nt::IntegerArrayTopic::ParamType &defaultValue = {})
{
    return GetTable(tableName)->GetIntegerArrayTopic(entryName).Subscribe(defaultValue);
}

inline static nt::DoubleSubscriber GetDoubleSubscriber(const std::string &tableName, const std::string &entryName, const nt::DoubleTopic::ParamType &defaultValue = {})
{
    return GetTable(tableName)->GetDoubleTopic(entryName).Subscribe(defaultValue);
}
inline static nt::DoubleArraySubscriber GetDoubleArraySubscriber(const std::string &tableName, const std::string &entryName, const nt::DoubleArrayTopic::ParamType &defaultValue = {})
{
    return GetTable(tableName)->GetDoubleArrayTopic(entryName).Subscribe(defaultValue);
}

inline static nt::StringSubscriber GetStringSubscriber(const std::string &tableName, const std::string &entryName, const nt::StringTopic::ParamType &defaultValue = {})
{
    return GetTable(tableName)->GetStringTopic(entryName).Subscribe(defaultValue);
}
inline static nt::StringArraySubscriber GetStringArraySubscriber(const std::string &tableName, const std::string &entryName, const nt::StringArrayTopic::ParamType &defaultValue = {})
{
    return GetTable(tableName)->GetStringArrayTopic(entryName).Subscribe(defaultValue);
}

template <typename T>
inline static nt::StructSubscriber<T> GetStructSubscriber(const std::string &tableName, const std::string &entryName, const T &defaultValue = {})
{
    return GetTable(tableName)->GetStructTopic<T>(entryName).Subscribe(defaultValue);
}
template <typename T>
inline static nt::StructArraySubscriber<T> GetStructArraySubscriber(const std::string &tableName, const std::string &entryName, const std::span<T> &defaultValue = {})
{
    return GetTable(tableName)->GetStructArrayTopic<T>(entryName).Subscribe(defaultValue);
}

// Publishers:

inline static nt::BooleanPublisher GetBooleanPublisher(const std::string &tableName, const std::string &entryName)
{
    return GetTable(tableName)->GetBooleanTopic(entryName).Publish();
}
inline static nt::BooleanArrayPublisher GetBooleanArrayPublisher(const std::string &tableName, const std::string &entryName)
{
    return GetTable(tableName)->GetBooleanArrayTopic(entryName).Publish();
}

inline static nt::IntegerPublisher GetIntegerPublisher(const std::string &tableName, const std::string &entryName)
{
    return GetTable(tableName)->GetIntegerTopic(entryName).Publish();
}
inline static nt::IntegerArrayPublisher GetIntegerArrayPublisher(const std::string &tableName, const std::string &entryName)
{
    return GetTable(tableName)->GetIntegerArrayTopic(entryName).Publish();
}

inline static nt::DoublePublisher GetDoublePublisher(const std::string &tableName, const std::string &entryName)
{
    return GetTable(tableName)->GetDoubleTopic(entryName).Publish();
}
inline static nt::DoubleArrayPublisher GetDoubleArrayPublisher(const std::string &tableName, const std::string &entryName)
{
    return GetTable(tableName)->GetDoubleArrayTopic(entryName).Publish();
}

inline static nt::StringPublisher GetStringPublisher(const std::string &tableName, const std::string &entryName)
{
    return GetTable(tableName)->GetStringTopic(entryName).Publish();
}
inline static nt::StringArrayPublisher GetStringArrayPublisher(const std::string &tableName, const std::string &entryName)
{
    return GetTable(tableName)->GetStringArrayTopic(entryName).Publish();
}

template <typename T>
inline static nt::StructPublisher<T> GetStructPublisher(const std::string &tableName, const std::string &entryName)
{
    return GetTable(tableName)->GetStructTopic<T>(entryName).Publish();
}
template <typename T>
inline static nt::StructArrayPublisher<T> GetStructArrayPublisher(const std::string &tableName, const std::string &entryName)
{
    return GetTable(tableName)->GetStructArrayTopic<T>(entryName).Publish();
}

// Entries:

inline static nt::BooleanEntry GetBooleanEntry(const std::string &tableName, const std::string &entryName, const nt::BooleanTopic::ParamType &defaultValue = {})
{
    return GetTable(tableName)->GetBooleanTopic(entryName).GetEntry(defaultValue);
}
inline static nt::BooleanArrayEntry GetBooleanArrayEntry(const std::string &tableName, const std::string &entryName, const nt::BooleanArrayTopic::ParamType &defaultValue = {})
{
    return GetTable(tableName)->GetBooleanArrayTopic(entryName).GetEntry(defaultValue);
}

inline static nt::IntegerEntry GetIntegerEntry(const std::string &tableName, const std::string &entryName, const int &defaultValue = 0)
{
    return GetTable(tableName)->GetIntegerTopic(entryName).GetEntry(defaultValue);
}
inline static nt::IntegerArrayEntry GetIntegerArrayEntry(const std::string &tableName, const std::string &entryName, const std::span<int64_t> &defaultValue = {})
{
    return GetTable(tableName)->GetIntegerArrayTopic(entryName).GetEntry(defaultValue);
}

inline static nt::DoubleEntry GetDoubleEntry(const std::string &tableName, const std::string &entryName, const double &defaultValue = 0.0)
{
    return GetTable(tableName)->GetDoubleTopic(entryName).GetEntry(defaultValue);
}
inline static nt::DoubleArrayEntry GetDoubleArrayEntry(const std::string &tableName, const std::string &entryName, const std::span<double> &defaultValue = {})
{
    return GetTable(tableName)->GetDoubleArrayTopic(entryName).GetEntry(defaultValue);
}

inline static nt::StringEntry GetStringEntry(const std::string &tableName, const std::string &entryName, const std::string &defaultValue = "")
{
    return GetTable(tableName)->GetStringTopic(entryName).GetEntry(defaultValue);
}
inline static nt::StringArrayEntry GetStringArrayEntry(const std::string &tableName, const std::string &entryName, const nt::StringArrayTopic::ParamType &defaultValue = {})
{
    return GetTable(tableName)->GetStringArrayTopic(entryName).GetEntry(defaultValue);
}

template <typename T>
inline static nt::StructEntry<T> GetStructEntry(const std::string &tableName, const std::string &entryName, const T &defaultValue = {})
{
    return GetTable(tableName)->GetStructTopic<T>(entryName).GetEntry(defaultValue);
}
template <typename T>
inline static nt::StructArrayEntry<T> GetStructArrayEntry(const std::string &tableName, const std::string &entryName, const std::span<T> &defaultValue = {})
{
    return GetTable(tableName)->GetStructArrayTopic<T>(entryName).GetEntry(defaultValue);
}
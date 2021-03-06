/*
* This file was generated by the CommonAPI Generators.
* Used org.genivi.commonapi.core 2.1.6.v20140519.
* Used org.franca.core 0.8.11.201401091023.
*
* This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0.
* If a copy of the MPL was not distributed with this file, You can obtain one at
* http://mozilla.org/MPL/2.0/.
*/
#include <org/genivi/EnhancedPositionService/EnhancedPositionStubDefault.h>

namespace org {
namespace genivi {
namespace EnhancedPositionService {

EnhancedPositionStubDefault::EnhancedPositionStubDefault():
        remoteEventHandler_(this),
        interfaceVersion_(EnhancedPosition::getInterfaceVersion()) {
}

const CommonAPI::Version& EnhancedPositionStubDefault::getInterfaceVersion(std::shared_ptr<CommonAPI::ClientId> clientId) {
    return interfaceVersion_;
}

EnhancedPositionStubRemoteEvent* EnhancedPositionStubDefault::initStubAdapter(const std::shared_ptr<EnhancedPositionStubAdapter>& stubAdapter) {
    CommonAPI::Stub<EnhancedPositionStubAdapter, EnhancedPositionStubRemoteEvent>::stubAdapter_ = stubAdapter;
    return &remoteEventHandler_;
}


/**
 * GetVersion = This method returns the API version implemented by the server
 *  application
 */
void EnhancedPositionStubDefault::GetVersion(const std::shared_ptr<CommonAPI::ClientId> clientId, EnhancedPositionServiceTypes::Version& version) {
    // Call old style methods in default 
    GetVersion(version);
}
void EnhancedPositionStubDefault::GetVersion(EnhancedPositionServiceTypes::Version& version) {
    // No operation in default
}

/**
 * GetPositionInfo = This method returns a given set of positioning data (e.g.
 *  Position, Course, Accuracy, Status, ... )
           Note: If a requested
 *  value is invalid, it's not returned to the client application
 */
void EnhancedPositionStubDefault::GetPositionInfo(const std::shared_ptr<CommonAPI::ClientId> clientId, EnhancedPositionServiceTypes::Bitmask valuesToReturn, EnhancedPositionServiceTypes::Timestamp& timestamp, EnhancedPositionServiceTypes::PositionInfo& data) {
    // Call old style methods in default 
    GetPositionInfo(valuesToReturn, timestamp, data);
}
void EnhancedPositionStubDefault::GetPositionInfo(EnhancedPositionServiceTypes::Bitmask valuesToReturn, EnhancedPositionServiceTypes::Timestamp& timestamp, EnhancedPositionServiceTypes::PositionInfo& data) {
    // No operation in default
}

/**
 * GetSatelliteInfo = This method returns information about the current satellite
 *  constellation
        Note: If a requested value is invalid, it's not
 *  returned to the client application
 */
void EnhancedPositionStubDefault::GetSatelliteInfo(const std::shared_ptr<CommonAPI::ClientId> clientId, EnhancedPositionServiceTypes::Timestamp& timestamp, EnhancedPositionServiceTypes::SatelliteInfo& satelliteInfo) {
    // Call old style methods in default 
    GetSatelliteInfo(timestamp, satelliteInfo);
}
void EnhancedPositionStubDefault::GetSatelliteInfo(EnhancedPositionServiceTypes::Timestamp& timestamp, EnhancedPositionServiceTypes::SatelliteInfo& satelliteInfo) {
    // No operation in default
}

/**
 * GetTime = This method returns UTC time and date.
            Note: If a
 *  requested value is invalid, it's not returned to the client application
 */
void EnhancedPositionStubDefault::GetTime(const std::shared_ptr<CommonAPI::ClientId> clientId, EnhancedPositionServiceTypes::Timestamp& timestamp, EnhancedPositionServiceTypes::TimeInfo& time) {
    // Call old style methods in default 
    GetTime(timestamp, time);
}
void EnhancedPositionStubDefault::GetTime(EnhancedPositionServiceTypes::Timestamp& timestamp, EnhancedPositionServiceTypes::TimeInfo& time) {
    // No operation in default
}


/**
 * PositionUpdate = This signal is called to notify a client application of a
 *  position change. The update frequency is implementation specific. The maximal
 *  allowed frequency is 10Hz
 */
void EnhancedPositionStubDefault::firePositionUpdateEvent(const EnhancedPositionServiceTypes::Bitmask& changedValues) {
    stubAdapter_->firePositionUpdateEvent(changedValues);
}


EnhancedPositionStubDefault::RemoteEventHandler::RemoteEventHandler(EnhancedPositionStubDefault* defaultStub):
        defaultStub_(defaultStub) {
}

} // namespace EnhancedPositionService
} // namespace genivi
} // namespace org

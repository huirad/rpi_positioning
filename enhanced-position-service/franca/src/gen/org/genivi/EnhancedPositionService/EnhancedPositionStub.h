/*
* This file was generated by the CommonAPI Generators.
* Used org.genivi.commonapi.core 2.1.6.v20140519.
* Used org.franca.core 0.8.11.201401091023.
*
* This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0.
* If a copy of the MPL was not distributed with this file, You can obtain one at
* http://mozilla.org/MPL/2.0/.
*/
/**
 * EnhancedPosition = This interface offers functionalities to retrieve the
 *  enhanced position of the vehicle
 */
#ifndef ORG_GENIVI_ENHANCEDPOSITIONSERVICE_Enhanced_Position_STUB_H_
#define ORG_GENIVI_ENHANCEDPOSITIONSERVICE_Enhanced_Position_STUB_H_



#include <org/genivi/EnhancedPositionService/EnhancedPositionServiceTypes.h>

#include "EnhancedPosition.h"

#if !defined (COMMONAPI_INTERNAL_COMPILATION)
#define COMMONAPI_INTERNAL_COMPILATION
#endif

#include <CommonAPI/InputStream.h>
#include <CommonAPI/OutputStream.h>
#include <CommonAPI/SerializableStruct.h>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <CommonAPI/Stub.h>

#undef COMMONAPI_INTERNAL_COMPILATION

namespace org {
namespace genivi {
namespace EnhancedPositionService {

/**
 * Receives messages from remote and handles all dispatching of deserialized calls
 * to a stub for the service EnhancedPosition. Also provides means to send broadcasts
 * and attribute-changed-notifications of observable attributes as defined by this service.
 * An application developer should not need to bother with this class.
 */
class EnhancedPositionStubAdapter: virtual public CommonAPI::StubAdapter, public EnhancedPosition {
 public:

    /**
     * Sends a broadcast event for PositionUpdate. Should not be called directly.
     * Instead, the "fire<broadcastName>Event" methods of the stub should be used.
     */
    virtual void firePositionUpdateEvent(const EnhancedPositionServiceTypes::Bitmask& changedValues) = 0;


    virtual void deactivateManagedInstances() = 0;
protected:
    /**
     * Defines properties for storing the ClientIds of clients / proxies that have
     * subscribed to the selective broadcasts
     */
};


/**
 * Defines the necessary callbacks to handle remote set events related to the attributes
 * defined in the IDL description for EnhancedPosition.
 * For each attribute two callbacks are defined:
 * - a verification callback that allows to verify the requested value and to prevent setting
 *   e.g. an invalid value ("onRemoteSet<AttributeName>").
 * - an action callback to do local work after the attribute value has been changed
 *   ("onRemote<AttributeName>Changed").
 *
 * This class and the one below are the ones an application developer needs to have
 * a look at if he wants to implement a service.
 */
class EnhancedPositionStubRemoteEvent {
 public:
    virtual ~EnhancedPositionStubRemoteEvent() { }

};


/**
 * Defines the interface that must be implemented by any class that should provide
 * the service EnhancedPosition to remote clients.
 * This class and the one above are the ones an application developer needs to have
 * a look at if he wants to implement a service.
 */
class EnhancedPositionStub: public virtual CommonAPI::Stub<EnhancedPositionStubAdapter, EnhancedPositionStubRemoteEvent> {
public:
    virtual ~EnhancedPositionStub() { }
    virtual const CommonAPI::Version& getInterfaceVersion(std::shared_ptr<CommonAPI::ClientId> clientId) = 0;


    /**
     * GetVersion = This method returns the API version implemented by the server
     *  application
     */
    /// This is the method that will be called on remote calls on the method GetVersion.
    virtual void GetVersion(const std::shared_ptr<CommonAPI::ClientId> clientId, EnhancedPositionServiceTypes::Version& version) = 0;
    /**
     * GetPositionInfo = This method returns a given set of positioning data (e.g.
     *  Position, Course, Accuracy, Status, ... )
               Note: If a requested
     *  value is invalid, it's not returned to the client application
     */
    /// This is the method that will be called on remote calls on the method GetPositionInfo.
    virtual void GetPositionInfo(const std::shared_ptr<CommonAPI::ClientId> clientId, EnhancedPositionServiceTypes::Bitmask valuesToReturn, EnhancedPositionServiceTypes::Timestamp& timestamp, EnhancedPositionServiceTypes::PositionInfo& data) = 0;
    /**
     * GetSatelliteInfo = This method returns information about the current satellite
     *  constellation
            Note: If a requested value is invalid, it's not
     *  returned to the client application
     */
    /// This is the method that will be called on remote calls on the method GetSatelliteInfo.
    virtual void GetSatelliteInfo(const std::shared_ptr<CommonAPI::ClientId> clientId, EnhancedPositionServiceTypes::Timestamp& timestamp, EnhancedPositionServiceTypes::SatelliteInfo& satelliteInfo) = 0;
    /**
     * GetTime = This method returns UTC time and date.
                Note: If a
     *  requested value is invalid, it's not returned to the client application
     */
    /// This is the method that will be called on remote calls on the method GetTime.
    virtual void GetTime(const std::shared_ptr<CommonAPI::ClientId> clientId, EnhancedPositionServiceTypes::Timestamp& timestamp, EnhancedPositionServiceTypes::TimeInfo& time) = 0;
    /**
     * PositionUpdate = This signal is called to notify a client application of a
     *  position change. The update frequency is implementation specific. The maximal
     *  allowed frequency is 10Hz
     */
    /// Sends a broadcast event for PositionUpdate.
    virtual void firePositionUpdateEvent(const EnhancedPositionServiceTypes::Bitmask& changedValues) = 0;

    using CommonAPI::Stub<EnhancedPositionStubAdapter, EnhancedPositionStubRemoteEvent>::initStubAdapter;
    typedef CommonAPI::Stub<EnhancedPositionStubAdapter, EnhancedPositionStubRemoteEvent>::StubAdapterType StubAdapterType;
    typedef CommonAPI::Stub<EnhancedPositionStubAdapter, EnhancedPositionStubRemoteEvent>::RemoteEventHandlerType RemoteEventHandlerType;
    typedef EnhancedPositionStubRemoteEvent RemoteEventType;
    typedef EnhancedPosition StubInterface;
};

} // namespace EnhancedPositionService
} // namespace genivi
} // namespace org

#endif // ORG_GENIVI_ENHANCEDPOSITIONSERVICE_Enhanced_Position_STUB_H_

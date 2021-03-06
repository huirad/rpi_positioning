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
 * Configuration = This interface allows a client application to set and retrieve
 *  configuration options
 */
#ifndef ORG_GENIVI_ENHANCEDPOSITIONSERVICE_Configuration_STUB_DEFAULT_H_
#define ORG_GENIVI_ENHANCEDPOSITIONSERVICE_Configuration_STUB_DEFAULT_H_


#include <org/genivi/EnhancedPositionService/ConfigurationStub.h>
#include <sstream>

namespace org {
namespace genivi {
namespace EnhancedPositionService {

/**
 * Provides a default implementation for ConfigurationStubRemoteEvent and
 * ConfigurationStub. Method callbacks have an empty implementation,
 * remote set calls on attributes will always change the value of the attribute
 * to the one received.
 *
 * Override this stub if you only want to provide a subset of the functionality
 * that would be defined for this service, and/or if you do not need any non-default
 * behaviour.
 */
class ConfigurationStubDefault : public virtual ConfigurationStub {
public:
    ConfigurationStubDefault();

    ConfigurationStubRemoteEvent* initStubAdapter(const std::shared_ptr<ConfigurationStubAdapter>& stubAdapter);

    const CommonAPI::Version& getInterfaceVersion(std::shared_ptr<CommonAPI::ClientId> clientId);

    virtual const EnhancedPositionServiceTypes::SatelliteSystem& getSatSystemAttribute();
    virtual const EnhancedPositionServiceTypes::SatelliteSystem& getSatSystemAttribute(const std::shared_ptr<CommonAPI::ClientId> clientId);
    virtual void setSatSystemAttribute(EnhancedPositionServiceTypes::SatelliteSystem value);
    virtual void setSatSystemAttribute(const std::shared_ptr<CommonAPI::ClientId> clientId, EnhancedPositionServiceTypes::SatelliteSystem value);
    virtual const int32_t& getUpdateIntervalAttribute();
    virtual const int32_t& getUpdateIntervalAttribute(const std::shared_ptr<CommonAPI::ClientId> clientId);
    virtual void setUpdateIntervalAttribute(int32_t value);
    virtual void setUpdateIntervalAttribute(const std::shared_ptr<CommonAPI::ClientId> clientId, int32_t value);

    /**
     * GetVersion = This method returns the API version implemented by the server
     *  application
     */
    virtual void GetVersion(const std::shared_ptr<CommonAPI::ClientId> clientId, EnhancedPositionServiceTypes::Version& version);
    virtual void GetVersion(EnhancedPositionServiceTypes::Version& version);

    /**
     * GetSupportedSatelliteSystems = This method returns a list of supported
     *  satellite systems
     */
    virtual void GetSupportedSatelliteSystems(const std::shared_ptr<CommonAPI::ClientId> clientId, std::vector<EnhancedPositionServiceTypes::SatelliteSystem>& satelliteSystems);
    virtual void GetSupportedSatelliteSystems(std::vector<EnhancedPositionServiceTypes::SatelliteSystem>& satelliteSystems);




protected:
    /**
     * SatSystem = satellite system (GPS, GLONASS, ...)
     */
    virtual bool trySetSatSystemAttribute(EnhancedPositionServiceTypes::SatelliteSystem value);
    virtual bool validateSatSystemAttributeRequestedValue(const EnhancedPositionServiceTypes::SatelliteSystem& value);
    virtual void onRemoteSatSystemAttributeChanged();
    /**
     * UpdateInterval = update interval
     */
    virtual bool trySetUpdateIntervalAttribute(int32_t value);
    virtual bool validateUpdateIntervalAttributeRequestedValue(const int32_t& value);
    virtual void onRemoteUpdateIntervalAttributeChanged();
    class RemoteEventHandler: public virtual ConfigurationStubRemoteEvent {
     public:
        RemoteEventHandler(ConfigurationStubDefault* defaultStub);

        /**
         * SatSystem = satellite system (GPS, GLONASS, ...)
         */
        virtual bool onRemoteSetSatSystemAttribute(EnhancedPositionServiceTypes::SatelliteSystem value);
        virtual bool onRemoteSetSatSystemAttribute(const std::shared_ptr<CommonAPI::ClientId> clientId, EnhancedPositionServiceTypes::SatelliteSystem value);
        virtual void onRemoteSatSystemAttributeChanged();

        /**
         * UpdateInterval = update interval
         */
        virtual bool onRemoteSetUpdateIntervalAttribute(int32_t value);
        virtual bool onRemoteSetUpdateIntervalAttribute(const std::shared_ptr<CommonAPI::ClientId> clientId, int32_t value);
        virtual void onRemoteUpdateIntervalAttributeChanged();


     private:
        ConfigurationStubDefault* defaultStub_;
    };
private:
    ConfigurationStubDefault::RemoteEventHandler remoteEventHandler_;

    /**
     * SatSystem = satellite system (GPS, GLONASS, ...)
     */
    EnhancedPositionServiceTypes::SatelliteSystem satSystemAttributeValue_;
    /**
     * UpdateInterval = update interval
     */
    int32_t updateIntervalAttributeValue_;

    CommonAPI::Version interfaceVersion_;
};

} // namespace EnhancedPositionService
} // namespace genivi
} // namespace org

#endif // ORG_GENIVI_ENHANCEDPOSITIONSERVICE_Configuration_STUB_DEFAULT_H_

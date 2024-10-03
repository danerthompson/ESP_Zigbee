const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
//const extend = require('zigbee-herdsman-converters/lib/extend');
const ota = require('zigbee-herdsman-converters/lib/ota');
const tuya = require('zigbee-herdsman-converters/lib/tuya');
const { } = require('zigbee-herdsman-converters/lib/tuya');
const utils = require('zigbee-herdsman-converters/lib/utils');
const globalStore = require('zigbee-herdsman-converters/lib/store');
const e = exposes.presets;
const ea = exposes.access;

const {Zcl, ZSpec} = require('zigbee-herdsman');
const {deviceAddCustomCluster} = require ('zigbee-herdsman-converters/lib/modernExtend');
const manufacturerOptions = {manufacturerCode: Zcl.ManufacturerCode.RESERVED_10};
const constants = require('zigbee-herdsman-converters/lib/constants');

// These are required to configure binding/attribute reporting in the front-end
const customExtend = {
  electricalCluster: () => deviceAddCustomCluster(
    'fieldUnitElectrical',
    {
      ID: 0xFC69,
      attributes: {
        measuredBattery: {
          ID: 0x0020,
          type: Zcl.DataType.INT16,
          manufacturerCode: Zcl.ManufacturerCode.RESERVED_10,   // Make sure this matches with nordic cluster definition
        },
      },
      commands: {},
      commandsResponse: {},
    },
  ),
};

const fLocal = {
  electrical: {
    cluster: 'fieldUnitElectrical',   // Name from cluster above
    type: ['readResponse', 'attributeReport'],
    convert: (model, msg, publish, options, meta) => {
      // Append received attribute reports to account for report containing multiple attributes in a single report
      const result = {};
      if (msg.data.hasOwnProperty('32')) {              // Decimal version of attribute ID
        const field_unit_battery_val = parseFloat(msg.data['32']) / 1000.0;
        result.field_unit_battery = field_unit_battery_val;           // Use the name of this property in the exposes (implement multiple attributes this way?)
      }
      return result;
    },
  },
};

const { identify, temperature, humidity, illuminance, occupancy, onOff } = require('zigbee-herdsman-converters/lib/modernExtend');

const definition = {
  zigbeeModel: ['ESP_Stickbee'],
  model: 'ESP_Stickbee',
  vendor: 'Stickman Solutions LLC',
  description: 'Custom definition for ESP_Stickbee',

  // These settings define the clusters used
  extend: [identify(), temperature(), humidity(), illuminance(), occupancy(), onOff(),
           customExtend.electricalCluster()],

  // These settings define what to do with incoming messages from ZigBee (attribute reports)
  fromZigbee: [fLocal.electrical],
  toZigbee: [],

  // These settings define what is exposed to MQTT and HomeAssistant
  exposes: [e.numeric('field_unit_battery', ea.STATE).withLabel('Battery').withUnit('V').withDescription('Battery voltage from ESP_StickBee electrical cluster')], // Name is property in fromZigbee object
  meta: {},

  // These settings set the configuration behavior when the device first joins the network
  configure: async (device, coordinatorEndpoint) => {
    const endpoint = device.getEndpoint(10);             // Endpoint on end device
    // BINDING STILL CAUSES DUPLICATE REPORTS, MAYBE TRY endpoint.unbind() like in bosch.ts? (UPDATE, MAYBE IT'S OKAY?)
    await reporting.bind(endpoint, coordinatorEndpoint, [
        'fieldUnitElectrical',
    ]);
    await endpoint.configureReporting('fieldUnitElectrical', [{
        attribute: 'measuredBattery',
        minimumReportInterval: 1,    // Used to be constants.repInterval.SECONDS_10
        maximumReportInterval: 0,    // Used to be constants.repInterval.HOUR
        reportableChange: 0,
    }], manufacturerOptions);
  },
};

module.exports = definition; 
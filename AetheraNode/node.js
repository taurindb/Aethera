//Aethera Client App

var socket = require('socket.io-client')('http://54.183.67.220'); //Public IP of your Server
var fs = require('fs');
var moment = require('moment');
var curl = require('curlrequest')
var sp = require('serialport')
var SerialPort = sp.SerialPort;
var serialport = new SerialPort('/dev/ttyACM0', { //Arduino Device ID
    baudrate: 57600,
    parser: sp.parsers.readline('\n')
});

serialport.on('open', function() {

    curl.request({
        url: 'http://freegeoip.net/json/'
    }, function(err, geodata) {
        geoip = JSON.parse(geodata);
        ip = geoip.ip;
        latitude = geoip.latitude;
        longitude = geoip.longitude;
        country = geoip.country_name;
        region = geoip.region_name;
        city = geoip.city;
        zipcode = geoip.zipcode;

        country = country.replace(/ /g,"_");
        
        var location = country + ', ' + city + ', ' + region + ', ' + latitude + ', ' + longitude + ',';
        var nodeName = country + '_' + latitude + '_' + longitude;
        var timestamp;
        var message = {};
        var environmentData;

        console.log('Logging Data in ' + country + ', ' + city + ' ' + region + '!');
        
        serialport.on('data', function(data) {
            var timeStamp = new moment().format('YYYY-MM-DD HH:mm:ss');
            environmentData = data.toString();
            message.clientName = nodeName;
            message.values = timeStamp + "," + environmentData;
            console.log("message = " + message.values);
            socket.emit("message", message);
            fs.appendFile(__dirname + "/" + nodeName + '.txt', message.values, function(err) {
                if (err) throw err;
                console.log('message was appended to file!');
            });
        });
    
    socket.emit(nodeName, message);
    });
});
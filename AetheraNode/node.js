//Aethera Client App

var socket = require('socket.io-client')('http://12.354.67.89'); //Public IP of your Server
var fs = require('fs');
var moment = require('moment');
var sp = require("serialport")
var SerialPort = sp.SerialPort;
var serialport = new SerialPort("/dev/ttyACM0", {  //Arduino Device ID
    baudrate: 57600,
    parser: sp.parsers.readline("\n")
});

serialport.on('open', function() {
    var nodeName = 'Node1';
    var timestamp;
    var message = {};
    var readData;

    console.log('Serial Port Open!');
    serialport.on('data', function(data) {
        var timeStamp = new moment().format('YYYY-MM-DD HH:mm:ss');
        readData = data.toString();
        message.clientName = nodeName;
        message.values = timeStamp + "," + readData;
        console.log("message = " + message.values);
        socket.emit("message", message);
        fs.appendFile(nodeName + '.txt', message.values, function(err) {
            if (err) throw err;
            console.log('message was appended to file!');
        });
    });
    socket.emit(nodeName, message);
});
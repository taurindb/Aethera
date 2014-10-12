var app = require('http').createServer(handler)
var io = require('socket.io')(app);
var fs = require('fs');

app.listen(80);

function handler (req, res) {
 var url = req.url; 
 fs.readFile(__dirname + url,
  function (err, data) {
    if (err) {
      res.writeHead(500);
      return res.end('Error loading index.html');
    }
    res.writeHead(200);
    res.end(data);
  });
}

io.on('connection', function (socket) {
  socket.on('message', function (data) {
    fs.appendFile(data.clientName + ".txt", data.values, function (err) {
     if (err) throw err;
     console.log(data.clientName +'data was appended to file!');
     console.log(data.values);
    fs.writeFile(data.clientName + "latest.txt", data.values, function (err) {
     if (err) throw err;
     });   
    });
  });
});


var http = require('http');
var fs = require('fs');
var url = require('url');
var querystring = require('querystring');
var ppmbin = require ('ppm-bin');
var mysql = require('mysql');
var con = mysql.createConnection({
  host: "localhost",
  user: "jhu20",
  password: "yuio3fzkd",
  database: "yingli"
});

con.connect(function(err) {
  if (err) throw err;
  console.log("Connected!");
});


// Create a server 
http.createServer( function (req, resp) {
		
	
	// Parse the request containing file name
	var pathname = url.parse(req.url).pathname;
	// Print the name of the file for which request is made
	console.log("Request for " + pathname + " received.");
	
	if (pathname == '/prefcalc') {
	  var query = querystring.parse(url.parse(req.url).query);
	  console.log("IP info " + JSON.stringify(query) + " received.");
	
	  var spawn = require('child_process').spawn;
	  var py = spawn('python', ['subnet3.py']);
	  var dataString = '';
	  py.stdout.on('data', function(data) {
		  dataString = data;
	  });
	  py.stdout.on('end', function() {
		  console.log("Going to write ", dataString, " into prefcalc");
		  fs.writeFile('prefcalc', dataString, function (err) {
			  if (err) {
				  console.log(err);
			  }
			  console.log("Data written successfully!");
			  // Read the requestedf file content from file system
			  fs.readFile(pathname.substr(1), function (err, data) {
				  if (err) {
					  console.log(err);
					  // HTTP Status: 404 : NOT FOUND
					  // Content Type: text/plain
					  resp.writeHead(404, {'Content-Type': 'text/html'});
				  } 
				  else {
					  // HTTP Status: 200 : OK
					  // Content Type: text/plain
					  resp.writeHead(200, {'Content-Type': 'text/html'});
			
					  // Write the content of the file to response body
					  resp.write(data.toString());
				  }
				  // Send the response body
				  resp.end();
			  });
		  });
	  });
	  py.stdin.write(JSON.stringify(query));
	  py.stdin.end();
  	}
	else if (pathname == '/result.png') {
	//how to load picture?
	  console.log('loading the image');
	  //testing ppm-bin
	  ppmbin.convert('result.ppm', 'result.png',function(err){;});
	  setTimeout(function() {
    		fs.readFile(pathname.substr(1), function (err, data) {
		console.log(pathname.substr(1));
		  if (err) {
			  console.log(err);
			  // HTTP Status: 404 : NOT FOUND
			  // Content Type: text/plain
			  resp.writeHead(404, {'Content-Type': 'text/html'});
		  } 
		  else {
			  // Page found
			  // HTTP Status: 200 : OK
			  // Content Type: text/plain
			  resp.writeHead(200, {'Content-Type': 'text/html'});
			
			  // Write the content of the file to response body
			  resp.write(data);
		  }
		  // Send the response body
		  resp.end();
	  	});
	  }, 1000);
	  
    	} 

	
	else if (pathname == '/update'){
		console.log('asking for picture updates');
		con.query("SELECT x,y,t FROM robotLocation WHERE robotID = 58 ORDER BY RLocID DESC LIMIT 2", 
		function (err, result, fields) {
    			if (err) throw err;
			resp.writeHead(200, {'Content-Type': 'text/html'});
			if(result[0].x == result[1].x && result[0].y == result[1].y && result[0].t == result[1].t){
				console.log("same");
				resp.write("same");
			}
			else{
				console.log("diff");
				resp.write("diff");			
			}
			resp.end();
  		});
		console.log('finish update');
	
	}	

	else if(pathname == '/move'){
	  var query = querystring.parse(url.parse(req.url).query);
	  console.log("IP info " + JSON.stringify(query) + " received.");
	  var dataString = '';
	  
	  var spawn = require('child_process').spawn;
	  var py = spawn('python', ['move.py']);
	  py.stdout.on('data', function(data) {
		  dataString = String(data);
		  console.log(dataString);
	  });
	  py.stdout.on('end', function() {
		resp.end();
	  });
	  py.stdin.write(JSON.stringify(query));
	  py.stdin.end();
	  
	}
	else {
	  // Read the requestedf file content from file system
	  fs.readFile(pathname.substr(1), function (err, data) {
		console.log(pathname.substr(1));
		  if (err) {
			  console.log(err);
			  // HTTP Status: 404 : NOT FOUND
			  // Content Type: text/plain
			  resp.writeHead(404, {'Content-Type': 'text/html'});
		  } 
		  else {
			  // Page found
			  // HTTP Status: 200 : OK
			  // Content Type: text/plain
			  resp.writeHead(200, {'Content-Type': 'text/html'});
			
			  // Write the content of the file to response body
			  resp.write(data);
		  }
		  // Send the response body
		  resp.end();
	  });
  }
}
).listen(7000);

// Console will print the message
console.log('Server running at http://localhost:8000/');

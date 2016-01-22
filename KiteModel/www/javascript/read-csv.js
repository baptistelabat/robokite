// Inspired from https://github.com/MounirMesselmeni/html-fileapi
function handleFiles(files) {
	// Check for the various File API support.
	if (window.FileReader) {
		// FileReader are supported.
		getAsText(files[0]);
	} else {
		alert('FileReader are not supported in this browser.');
	}
}

function getAsText(fileToRead) {
	var reader = new FileReader();
	// Handle errors load
	reader.onload = loadHandler;
	reader.onerror = errorHandler;
	// Read file into memory as UTF-8      
	reader.readAsText(fileToRead);
}

function loadHandler(event) {
	var csv = event.target.result;
	processData(csv);             
}

function processData(csv) {
    var allTextLines = csv.split(/\r\n|\n/);
    console.log(allTextLines.length);
    var lines = [];
    for (i=0;i<12;i++)
    {
      allTextLines.shift()
    }
    while (allTextLines.length) {
        lines.push(allTextLines.shift().split('  '));
    }
	console.log(lines[50][3]);
	drawOutput(lines);
}

function errorHandler(evt) {
	if(evt.target.error.name == "NotReadableError") {
		alert("Canno't read file !");
	}
}

function drawOutput(lines){
  
  
	//Clear previous data
	document.getElementById("output").innerHTML = "";
	var table = document.createElement("table");
	for (var i = 0; i < lines.length-1; i++) {
    i_alpha = 1;
    i_CL    = 2;
    i_CD    = 3;
    alpha_deg[i] = parseFloat(lines[i][i_alpha]);
    CL[i] = parseFloat(lines[i][i_CL]);
    CD[i] = parseFloat(lines[i][i_CD]);
		var row = table.insertRow(-1);
		for (var j = 0; j < Math.min(1,lines[i].length); j++) {
			var firstNameCell = row.insertCell(-1);
			firstNameCell.appendChild(document.createTextNode(lines[i][j]));
		}
	}
	document.getElementById("output").appendChild(table);
}

const proj4 = require('proj4');
const proj4list = require('./list.min.js');
const fs = require('fs');
const path = require('path');


const wgs84 = proj4.Proj('EPSG:4326');
const test = proj4.Proj('EPSG:32618');

console.log(test);
proj4.defs([proj4list["EPSG:32611"]]);  //"EPSG:32611":["EPSG:32611","+proj=utm +zone=11 +datum=WGS84 +units=m +no_defs "]);
//proj4.defs(["EPSG:32611","+proj=utm +zone=11 +datum=WGS84 +units=m +no_defs "]);

let gpsFix = [37.102491666666666, -76.3879, 123.0]; // 37.102491666666666,-76.38799  Alt: 123
let gpsOrientation = [0., 0., 0.]; // Roll, pitch, yaw in degrees


function readFilesInDirectory(directoryPath) {
  const files = fs.readdirSync(directoryPath);
  files.forEach(file => {
    const filePath = path.join(directoryPath, file);
    const fileData = fs.readFileSync(filePath, 'utf-8');
    const rows = fileData.split('\n');
    if ( file.endsWith('.csv') ){
        let number = 0;
        rows.forEach(row => {
            const columns = row.split(',');
            
            // Extract values from columns 3 to 5 (assuming zero-based indexing)
            const extractedValues = columns.slice(3, 6);
            
            // Do something with the extracted values
            let newExtarctedValues = [];
            extractedValues.forEach(extractedValue => {
                newExtarctedValues.push(parseFloat(extractedValue));
            });
            if (number > 0 && newExtarctedValues.length > 0){
                coords = convertCoordinates(newExtarctedValues, gpsFix, gpsOrientation);
                console.log(coords);
            }
            number += 1;
        });
    }
    
  });
}

function getUTMZone(sensorLocation) {
    console.log("lon " + sensorLocation[0] + " lat " + sensorLocation[1] + " alt " + sensorLocation[2]);
    const lon = sensorLocation[0];
    let zoneNumber = Math.floor((lon + 180) / 6) + 1;

    if (zoneNumber > 60) zoneNumber = 60;

    // Zone letter code
    let zoneLetter = 'N';  // default is northern
    const lat = sensorLocation[1];

    if (lat < 0)  // if latitude is below 0, it's in the southern hemisphere
        zoneLetter = 'S';

    return 'EPSG:32' + (zoneLetter == 'S' ? '7' : '6') +
        zoneNumber.toString().padStart(2, '0');
}
  
  // Apply rotations
function applyRPY(point, roll, pitch, yaw) {
    const radRoll = roll * Math.PI / 180;  // Convert to radians
    const radPitch = pitch * Math.PI / 180;
    const radYaw = yaw * Math.PI / 180;

    // Rotation matrices
    const rollMatrix = [
        [1, 0, 0], [0, Math.cos(radRoll), -Math.sin(radRoll)],
        [0, Math.sin(radRoll), Math.cos(radRoll)]
    ];

    const pitchMatrix = [
        [Math.cos(radPitch), 0, Math.sin(radPitch)], [0, 1, 0],
        [-Math.sin(radPitch), 0, Math.cos(radPitch)]
    ];

    const yawMatrix = [
        [Math.cos(radYaw), -Math.sin(radYaw), 0],
        [Math.sin(radYaw), Math.cos(radYaw), 0], [0, 0, 1]
    ];
  
    // Apply the rotations
    const rotatedPointRoll = multiplyMatrixVector(rollMatrix, point);
    const rotatedPointRollPitch =
        multiplyMatrixVector(pitchMatrix, rotatedPointRoll);
    const rotatedPoint = multiplyMatrixVector(yawMatrix, rotatedPointRollPitch);
  
    return rotatedPoint;
}
  
  // Function to multiply a 3x3 matrix by a 3D vector
function multiplyMatrixVector(matrix, vector) {
    const result = [0, 0, 0];
  
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        result[i] += matrix[i][j] * vector[j];
      }
    }
    return result;
}
  
  // Convert Lidar point cloud based coordinates to WGS84 coordinates
  // according the Lidar location and its orientation
function convertCoordinates(point, sensorLocation, orientation) {
    // Verify that sensorLocation is valid
    if (sensorLocation.length !== 3) {
        throw new Error('sensorLocation must be an array with three elements: [lat, lon, alt]');
    }
    const lat = sensorLocation[1], lon = sensorLocation[0];
    if (lat < -90 || lat > 90 || lon < -180 || lon > 180) {
        throw new Error('Invalid sensorLocation. Latitude must be between -90 and 90, and longitude must be between -180 and 180');
    }
  
    // Roll, Pitch, Yaw rotations
    const roll = orientation[0], pitch = orientation[1], yaw = orientation[2];
  
    // Apply the rotation matrices in the Roll, Pitch, Yaw order
    const rotatedPoint = applyRPY(point, roll, pitch, yaw);
  
    // Get the UTM zone for the sensor location
    const utmZone = getUTMZone(sensorLocation);
    console.log("UTM zone: " + utmZone);
    //const utm = proj4.Proj(utmZone);
  
    // // Convert sensor location to UTM using Proj4js
    // const utmSensorLocation = proj4(wgs84, utm, [sensorLocation[0], sensorLocation[1]]);
  
    // // Shift rotated point with sensor location
    // const utmPoint = [
    //     rotatedPoint[0] + utmSensorLocation[0],
    //     rotatedPoint[1] + utmSensorLocation[1],
    //     rotatedPoint[2] + utmSensorLocation[2]
    // ];
  
    // console.log("shift rotated utm point: " + utmPoint);
   
    // // Convert utmPoint from UTM to WGS84
    // //const wgs84Coords = proj4(utm, wgs84, utmPoint);
  
    // return wgs84Coords;
}



// Usage example
const directoryPath = './FT20 Nasa 13 June_ Landing Frames';
readFilesInDirectory(directoryPath);
//convertCoordinates([ -18.2634, -16.5602, -1.58001 ], gpsFix, gpsOrientation);

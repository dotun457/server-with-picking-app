import '@kitware/vtk.js/Rendering/Profiles/Geometry';

import vtkDataArray from '@kitware/vtk.js/Common/Core/DataArray';
import vtkPoints from '@kitware/vtk.js/Common/Core/Points';
import vtkPolyData from '@kitware/vtk.js/Common/DataModel/PolyData';
import vtkActor from '@kitware/vtk.js/Rendering/Core/Actor';
import vtkColorTransferFunction from '@kitware/vtk.js/Rendering/Core/ColorTransferFunction';
import vtkColorMaps from '@kitware/vtk.js/Rendering/Core/ColorTransferFunction/ColorMaps';
import vtkMapper from '@kitware/vtk.js/Rendering/Core/Mapper';
import vtkPointPicker from '@kitware/vtk.js/Rendering/Core/PointPicker';
import vtkFullScreenRenderWindow from '@kitware/vtk.js/Rendering/Misc/FullScreenRenderWindow';
import vtkSphereSource from '@kitware/vtk.js/Filters/Sources/SphereSource';
import React, {useEffect, useRef} from 'react';
import proj4 from 'proj4';
import proj4list from './list.min'

var cnt = 0;
let frameId = -1;
let gpsFix = [-76.3879, 37.102491666666666, 4.468368]; // 37.102491666666666,-76.38799  Alt: 123
let gpsOrientation = [0., 0., 0.]; // Roll, pitch, yaw in degrees
let pickingSphere = null;
let sphereMapper = null;
let sphereActor = null;

// Define WGS84 Projection (lat, lon)
const wgs84 = proj4.Proj('EPSG:4326');
proj4.defs([
  proj4list["EPSG:32611"], proj4list["EPSG:32618"]]
);


// Get the UTM zone of the sensor - we need it to apply the translation
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
  const utm = proj4.Proj(utmZone);

  // Convert sensor location to UTM using Proj4js
  const utmSensorLocation = proj4(wgs84, utm, [sensorLocation[0], sensorLocation[1]]);

  // Shift rotated point with sensor location
  const utmPoint = [
      rotatedPoint[0] + utmSensorLocation[0],
      rotatedPoint[1] + utmSensorLocation[1],
      rotatedPoint[2] + utmSensorLocation[2]
  ];

  console.log("shift rotated utm point: " + utmPoint);

  // Convert utmPoint from UTM to WGS84
  const wgs84Coords = proj4(utm, wgs84, utmPoint);

  return wgs84Coords;
}


function VtkSocketTest() {
  const renderWindowContainerRef = useRef(null);
  const context = useRef(null);
  const websocketRef = useRef(null);
  const pointsRef = useRef(null);
  const polyDataRef = useRef(null);
  const mapperRef = useRef(null);

  useEffect(() => {
    if (!context.current) {
      // Set up vtk.js rendering
      const fullScreenRenderWindow = vtkFullScreenRenderWindow.newInstance({
        rootContainer: renderWindowContainerRef.current,
        background: [0, 0, 0],  // Set background color
      });
      const renderer = fullScreenRenderWindow.getRenderer();
      const renderWindow = fullScreenRenderWindow.getRenderWindow();

      // Create vtk.js objects for lidar visualization
      const actor = vtkActor.newInstance();
      const mapper = vtkMapper.newInstance({
        interpolateScalarsBeforeMapping: false,
        useLookupTableScalarRange: true,
        scalarVisibility: true
      });
      actor.getProperty().setPointSize(1);
      actor.setMapper(mapper);
      renderer.addActor(actor);

      const polyData = vtkPolyData.newInstance();
      const points = vtkPoints.newInstance();
      const scalars = vtkDataArray.newInstance({
        numberOfComponents: 1,
        values: [],  // Initialize with empty array
      });
      points.setData(polyData);
      polyData.setPoints(points);
      polyData.getPointData().setScalars(scalars);

      // should I reset the camera here or in the updatePolyData function?
      //  Reset the camera to view the entire scene

      pointsRef.current = points;
      polyDataRef.current = polyData;
      mapperRef.current = mapper;

      renderWindow.render();

      context.current = { fullScreenRenderWindow, renderWindow, renderer };

      const picker = vtkPointPicker.newInstance();
      picker.setPickFromList(1);
      picker.initializePickList();
      picker.addPickList(actor);

      // Pick on mouse right click
      renderWindow.getInteractor().onRightButtonPress((callData) => {
        if (renderer !== callData.pokedRenderer) {
          return;
        }

        const pos = callData.position;
        const point = [pos.x, pos.y, 0.0];
        picker.pick(point, renderer);

        if (picker.getActors().length > 0) {
          const pickedPoints = picker.getPickedPositions();
          if (pickedPoints.length > 0) {
            const pickedPoint = pickedPoints[0];
            const pickedPointId = picker.getPointId();
            //console.log(`Pick on screen at: ${point}`);
            console.log(`Picked point: ${pickedPointId}, coordinates: ${pickedPoint}`);
            try {
              console.log(`GPS fix: ${gpsFix}`);
              let gpsCoords =
                  convertCoordinates(pickedPoint, gpsFix, gpsOrientation);
              console.log(`GPS coordinates: ${gpsCoords}`);

              pickingSphere = vtkSphereSource.newInstance();
              pickingSphere.setRadius(0.5);
              pickingSphere.setCenter(pickedPoint);

              if (!sphereMapper) {
                sphereMapper = vtkMapper.newInstance();
                sphereMapper.setInputData(pickingSphere.getOutputData());
                sphereActor = vtkActor.newInstance();
                sphereActor.setMapper(sphereMapper);
                sphereActor.getProperty().setColor(1.0, 0.0, 0.0);
                renderer.addActor(sphereActor);
              }
              sphereMapper.setInputData(pickingSphere.getOutputData());

              window.open('https://www.google.com/maps/place/'+gpsCoords[1]+','+gpsCoords[0], '_blank', 'noreferrer');
            } catch (e) {
            }
          }
        }
        renderWindow.render();
      });

      // different way of constructing the polydata objet from the commented
      // code above
      const updatePolyData =
          (pclData, track) => {
            const numPoints = pclData.length;
            if (numPoints === 0) return;
            const intensityArray = new Uint16Array(numPoints);

            const polyData = vtkPolyData.newInstance();
            const points = vtkPoints.newInstance();
            points.setNumberOfPoints(numPoints);

            pclData.forEach((item, index) => {
              const [xVal, yVal, zVal, intensityVal] = item;
              points.setPoint(
                index, isNaN(xVal) ? 0 : xVal, isNaN(yVal) ? 0 : yVal,
                  isNaN(zVal) ? 0 : zVal);
              intensityArray[index] = intensityVal;
            });
            polyData.setPoints(points);

            const vertices = new Uint32Array(numPoints + 1);
            vertices[0] = numPoints;
            for (let i = 0; i < numPoints; i++) {
              vertices[i + 1] = i;
            }
            polyData.getVerts().setData(vertices);

            const intensityDataArray = vtkDataArray.newInstance({
              name: 'intensity',
              numberOfComponents: 1,
              values: intensityArray,
            });
            polyData.getPointData().setScalars(intensityDataArray);

            var intensityRange = intensityDataArray.getRange();

            var colorMap = vtkColorTransferFunction.newInstance();
            colorMap.applyColorMap(
                vtkColorMaps.getPresetByName('Viridis (matplotlib)'));
            colorMap.setMappingRange(intensityRange[0], intensityRange[1]);
            colorMap.updateRange();
            mapper.setLookupTable(colorMap);

            //polyData.modified();
            mapper.setInputData(polyData);
            if (cnt === 0)
            {
              renderer.resetCamera();
              console.log("reset cam");
            }
            renderer.resetCameraClippingRange();
            cnt++;
            renderWindow.render();
          }

      // For the test we just create a set of 10k points
      const nbTestPoints = 10;
      var testData = [];
      for (var i = 0; i < nbTestPoints; i++) {
        var x = Math.random();
        var y = Math.random();
        var z = Math.random();
        var intensity = 255;
        testData.push([x, y, z, intensity]);
      }

      // and we animate it every 10ms
      function animationCallback() {
        for (var i = 0; i < nbTestPoints; i++) {
          testData[i][0] += (Math.random() - 0.5) * 0.01;
          testData[i][1] += (Math.random() - 0.5) * 0.01;
          testData[i][2] += (Math.random() - 0.5) * 0.01;
        }
        updatePolyData(testData);
      }
      //setInterval(animationCallback, 10);

      // Create a WebSocket connection
      const socket = new WebSocket('ws://127.0.0.1:8765');
      websocketRef.current = socket;

      socket.onopen = () => {
        console.log('WebSocket connection established');
      };

      socket.onmessage = (event) => {
        //console.log("message received");
        const pclData = JSON.parse(event.data);
        if (frameId === -1)
        {
          gpsFix = pclData;
          console.log("GPS Fix received: " + gpsFix);
        }
        else if (frameId == 0){
          console.log("Frame " + frameId + " with " + pclData.length + " points")
          updatePolyData(pclData, true);
        }
        else
        {
          console.log("Frame " + frameId + " with " + pclData.length + " points")
          updatePolyData(pclData, false);
        }
        frameId = frameId + 1
      };

      socket.onclose = () => {
        console.log('WebSocket connection closed');
      };

      // Clean up WebSocket connection when component unmounts
      return () => {
        //socket.close();
      };
    }
  }, [renderWindowContainerRef]);

  const parseServerData = (data) => {
    return data;
    // Replace NaN values with null
    const sanitizedData =
        data.map((item) => item.map((value) => (isNaN(value) ? null : value)));
    return sanitizedData;
  };

  return <div ref = { renderWindowContainerRef } />;
};


export default VtkSocketTest;

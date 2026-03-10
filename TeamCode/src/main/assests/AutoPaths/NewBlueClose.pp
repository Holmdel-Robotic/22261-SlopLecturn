{
  "version": "1.8.0",
  "header": {
    "info": "Created with Pedro Pathing Plus Visualizer",
    "copyright": "Copyright 2026 Matthew Allen. Licensed under the Modified Apache License, Version 2.0.",
    "link": "https://github.com/Mallen220/PedroPathingPlusVisualizer"
  },
  "startPoint": {
    "x": 48,
    "y": 10,
    "heading": "linear",
    "startDeg": -176.49840991746987,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "mmdy49yl-9q9xv0",
      "name": "",
      "endPoint": {
        "x": 24,
        "y": 36,
        "heading": "tangential",
        "reverse": false,
        "degrees": 180,
        "targetX": 144,
        "targetY": 144,
        "startDeg": 180,
        "endDeg": 90
      },
      "controlPoints": [
        {
          "x": 24,
          "y": 8.531428571428558
        }
      ],
      "color": "#D8B99C",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "eventMarkers": []
    },
    {
      "id": "mml55fzc-b1lzdp",
      "name": "",
      "endPoint": {
        "x": 48,
        "y": 10,
        "heading": "linear",
        "reverse": false,
        "startDeg": 90,
        "endDeg": 180
      },
      "controlPoints": [],
      "color": "#87689D",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mml57mpp-u1cou3",
      "name": "",
      "endPoint": {
        "x": 10,
        "y": 10,
        "heading": "constant",
        "startDeg": 180,
        "endDeg": 180,
        "degrees": 180
      },
      "controlPoints": [],
      "color": "#B7699D",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mml57z0y-19jx5d",
      "name": "",
      "endPoint": {
        "x": 48,
        "y": 10,
        "heading": "constant",
        "degrees": 180
      },
      "controlPoints": [],
      "color": "#BAB878",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "mmdy49yl-9q9xv0"
    },
    {
      "kind": "path",
      "lineId": "mml55fzc-b1lzdp"
    },
    {
      "kind": "path",
      "lineId": "mml57mpp-u1cou3"
    },
    {
      "kind": "path",
      "lineId": "mml57z0y-19jx5d"
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 144,
          "y": 69.5
        },
        {
          "x": 144,
          "y": 144
        },
        {
          "x": 119,
          "y": 144
        },
        {
          "x": 137.5,
          "y": 119
        },
        {
          "x": 137.5,
          "y": 69.5
        }
      ],
      "color": "#dc2626",
      "fillColor": "#fca5a5",
      "type": "obstacle"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 6.5,
          "y": 119
        },
        {
          "x": 25,
          "y": 144
        },
        {
          "x": 0,
          "y": 144
        },
        {
          "x": 0,
          "y": 69.5
        },
        {
          "x": 6.5,
          "y": 69.5
        }
      ],
      "color": "#0b08d9",
      "fillColor": "#fca5a5",
      "type": "obstacle"
    }
  ],
  "extraData": {}
}
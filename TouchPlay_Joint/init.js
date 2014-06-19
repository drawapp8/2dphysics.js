/*
*
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Copyright (c) 2014, Samsung Electronics Co. Ltd.*/


var SCALE = 10;
var cirList = [];
var jointList = [];
var touchList = [];
var numOfCir = 225;
var cirCordX = [];
var cirCordY = [];
var arrListX = [];
var arrListY = [];
var clusterList = [];
var tX = 0, tY = 0;
var w = window.innerWidth/(2 * SCALE);
var h = window.innerHeight/SCALE;
var fDir = 1;
var destroyList = [];
var globalID = 0;
var touchBall = null;
var touchSize = (w + h)/20;
var loadFlag = 0;
var initForceVal = 1;
var initArrange = 1;
var SBALL = 0x0001;
var NBALL = 0x0002;
var TBALL = 0x0004;
var frameCount = 0;
var SamplePeriod = 60;
var msAccumulator = 0;

//var w = screen.width/20;
//alert(window.innerWidth + ":" + innerHeight);
var w = window.innerWidth/20;
//var h = screen.height/12;
var h = window.innerHeight/10;

//var w = window.innerWidth/20; //screen.width/20;
//var h = window.innerWidth/12;//screen.height/12;


var pPosition = [-w/1.05, h/2];
var food = 0;

//Bodies

var m_woodBodyLeft = null;
var m_woodBodyRight = null;
var playerBall = null;
//var evilBall = null;

//Init values

var isPaused = false;
var initialVelocity =  null;
var velocitySpeed = 30;
var magnetPower = 10;
var no_of_initial_foodBalls = 120;
var SCALE = 10;
var glassPieces = 10;
var DEGTORAD = 0.0174532925;
var RADTODEG = 57.2957795;
var blastPower = 20; //Blast power of Glass
var playerHealth = 100;
var foodBallVelocity = 3;
var eatCount = 0;
var broke = 0;
var evilSpeed = 15;
var mCount = 1;
var lCount = 1;
var evilBallRadius = h/35;
var playerBallRadius = (w+h)/70;
var padding = h/60 - (w/3.07 - w/3.1) + 5;
var freeze = false;
var yellowCount = 100;

//Colors

var innerBrickColor = 'rgba(96, 96, 96, 1)';
var outerBrickColor = 'rgba(96, 0, 0, 1)';
var glassColor = 'rgba(0, 128, 0, 1)';
var evilColor = 'rgba(192, 0, 0, 1)';
var foodColor = 'rgba(255, 255, 0, 1)';
var iceColor = 'rgba(0, 255, 255, 1)';
var magnetColor = 'rgba(0, 0, 255, 1)';
var lightningColor = 'rgba(0, 64, 0, 1)';
var poisonColor = 'rgba(255, 32, 32, 1)';
var woodColor = 'rgba(122, 71, 0, 1)';
var darkRedBrickColor = "#400000"; 
var lightRedBrickColor = "#600000";
var darkCementBrickColor = "#606060";
var lightCementBrickColor = "#909090";
var playerColorPink = 'rgba(255, 255, 0, 1.0)';
var playerColorBlue = 'rgba(0, 255, 255, 1.0)';
var playerColorYellow = 'rgba(35, 200, 35, 1.0)';
var touchColor =  'rgba(255, 255, 255, 1.0)';
//Collision Filtering flags

var PLAYER = 0x0001;
var FOOD = 0x0002;
var EVIL = 0x0004;
var STATIC_OBSTACLE = 0x0006;
var GROUND_OBSTACLE = 0x0010;
var MOVING_OBSTACLE = 0x0012;

function init() 
{
	if (window.testRunner || window.layoutTestController) 
	{
         testRunner.waitUntilDone();
         testRunner.dumpAsText();
     }
    if(typeof(WebBox2D) === "undefined") 
        Box2DWebLoader(initPage);
    else 
	{
        Box2D = WebBox2D;
        initPage();
    }
    
    if(navigator.platform.indexOf("arm") !== -1) // set scale for Tizen / PC
        document.body.style.zoom = 1.0;
    else
        document.body.style.zoom = 1.0;
}

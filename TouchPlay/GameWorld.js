/*
*
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Copyright (c) 2014, Samsung Electronics Co. Ltd.*/

window.requestAnimFrame = (function(){
          return  window.requestAnimationFrame       || 
                  window.webkitRequestAnimationFrame || 
                  window.mozRequestAnimationFrame    || 
                  window.oRequestAnimationFrame      || 
                  window.msRequestAnimationFrame     ||
                  function(/* function */ callback, /* DOMElement */ element){
                    window.setTimeout(callback, 1000 / 60);
                  };
    })();
 
var fancyDraw = new FancyDraw();

window.addEventListener('tizenhwkey', function(e) {
    if(e.keyName == "back")
        tizen.application.getCurrentApplication().exit();
});

function initPage() 
{
	var	b2Vec2 = Box2D.Common.Math.b2Vec2,
		b2BodyDef = Box2D.Dynamics.b2BodyDef,
		b2Body = Box2D.Dynamics.b2Body,
		b2FixtureDef = Box2D.Dynamics.b2FixtureDef,
		b2Fixture = Box2D.Dynamics.b2Fixture,
		b2RevoluteJointDef = Box2D.Dynamics.Joints.b2RevoluteJointDef,
		b2DistanceJointDef = Box2D.Dynamics.Joints.b2DistanceJointDef,
		b2PrismaticJointDef = Box2D.Dynamics.Joints.b2PrismaticJointDef,
		b2PulleyJointDef = Box2D.Dynamics.Joints.b2PulleyJointDef,
		b2RopeJointDef = Box2D.Dynamics.Joints.b2RopeJointDef,
		b2WheelJointDef = Box2D.Dynamics.Joints.b2WheelJointDef,
		b2GearJointDef = Box2D.Dynamics.Joints.b2GearJointDef,
		b2WeldJointDef = Box2D.Dynamics.Joints.b2WeldJointDef,
		b2World = Box2D.Dynamics.b2World,
		b2MassData = Box2D.Collision.Shapes.b2MassData,
		b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape,
		b2CircleShape = Box2D.Collision.Shapes.b2CircleShape,
		b2EdgeShape = Box2D.Collision.Shapes.b2EdgeShape,
		b2DebugDraw = Box2D.Dynamics.b2DebugDraw,	
	    b2Color =  Box2D.Common.b2Color;
		
	world = new b2World(
	   	new b2Vec2(0, -10),	//gravity
		false				//allow sleep
	);


var mouse_input = function (evt) {
	evt.preventDefault();
}	
		
//document.addEventListener('contextmenu', mouse_input, false);
document.addEventListener('mousemove',moveCircle,false);
document.addEventListener('touchmove',moveCircle,false);


	function moveCircle(e)
	{
		var ecX, ecY;
		if(navigator.platform.indexOf("arm") !== -1)
		{
			ecX = e.touches[0].clientX;
			ecY = e.touches[0].clientY; 
		}
		else
		{
			ecX = e.clientX;
			ecY = e.clientY
		}
		tX = w - ecX / SCALE;
		tY = h - ecY / SCALE;
		if (tX>-w && tX<w && tY>0 && tY<h)
		    touchBall.SetTransform (new b2Vec2(tX, tY), 0);
		pushPull();
	}
	
	touchPlay();
	//Box2D Code ends here
	var width = window.innerWidth;
	var height = window.innerHeight;
	fancyDraw.SetCanvas("canvas");
	
	// move origin down and invert y (to match coords used in C++ Testbed)
	fancyDraw.Translate(width/2, height);
	fancyDraw.Rotate(Math.PI);
	fancyDraw.Scale(SCALE);
	fancyDraw.b2Shape = Box2D.Collision.Shapes.b2Shape;
	fancyDraw.b2Math = Box2D.Common.Math.b2Math;
	requestAnimFrame(draw);
}

function touchPlay()
{

var	b2Vec2 = Box2D.Common.Math.b2Vec2,
		b2BodyDef = Box2D.Dynamics.b2BodyDef,
		b2Body = Box2D.Dynamics.b2Body,
		b2FixtureDef = Box2D.Dynamics.b2FixtureDef,
		b2Fixture = Box2D.Dynamics.b2Fixture,
		b2RevoluteJointDef = Box2D.Dynamics.Joints.b2RevoluteJointDef,
		b2DistanceJointDef = Box2D.Dynamics.Joints.b2DistanceJointDef,
		b2PrismaticJointDef = Box2D.Dynamics.Joints.b2PrismaticJointDef,
		b2PulleyJointDef = Box2D.Dynamics.Joints.b2PulleyJointDef,
		b2RopeJointDef = Box2D.Dynamics.Joints.b2RopeJointDef,
		b2WheelJointDef = Box2D.Dynamics.Joints.b2WheelJointDef,
		b2GearJointDef = Box2D.Dynamics.Joints.b2GearJointDef,
		b2WeldJointDef = Box2D.Dynamics.Joints.b2WeldJointDef,
		b2World = Box2D.Dynamics.b2World,
		b2MassData = Box2D.Collision.Shapes.b2MassData,
		b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape,
		b2CircleShape = Box2D.Collision.Shapes.b2CircleShape,
		b2EdgeShape = Box2D.Collision.Shapes.b2EdgeShape,
		b2DebugDraw = Box2D.Dynamics.b2DebugDraw,	
	    b2Color =  Box2D.Common.b2Color;


var fd = new b2FixtureDef;
var bd = new b2BodyDef;
var touchCircle = new b2CircleShape;
{	
	bd.position = new b2Vec2(tX, tY);
	bd.type = b2Body.b2_dynamicBody;
	touchBall = world.CreateBody(bd);
	touchBall.type = 'touch';
	touchBall.SetGravityScale(0);
	touchBall.SetSleepingAllowed(0);
	touchBall.color = touchColor;
	touchCircle.m_radius = touchSize;
	fd.shape = touchCircle;
	fd.density = 100;
	fd.restitution = 1;
	fd.filter.categoryBits = NBALL;
	//fd.filter.maskBits = NBALL;
	fd.filter.groupIndex = -1;
	touchBall.CreateFixture(fd);
}

	var bd = new b2BodyDef;
	var box = new b2PolygonShape;
	var fd = new b2FixtureDef;
	var nRow = Math.floor(Math.sqrt(numOfCir));
	var nTotal = nRow * nRow;
	var xp, xy;
	var xSpace = 1.8*w/nRow;
	var ySpace = 0.9*h/nRow;
	var ballColor;
	var ballRadius;
	var plusOrMinus;
	//alert(w + ":" + h + ":rows " + nRow );
	for(var i = 1; i <= nRow; i++)
	{
		for(var j = 1; j <= nRow; j++)
		{
		
			xp = w - xSpace * j - w/30;
			yp = h - ySpace * (i) - h/15 + 3;
			//alert("xp:"+xp + " w:"+w + " xSpace:" + xSpace);
			cirCordX.push(xp);
			cirCordY.push(yp);	
			
			var bC = (Math.floor(Math.random()*10)) % 3;
			var bR = (Math.floor(Math.random()*10)) %3;
			
			switch(bC)
			{
				case 0:
					ballColor = playerColorPink;
					break;
				case 1:
					ballColor = playerColorBlue;
					break;
				case 2:
					ballColor = playerColorYellow;
					break;
				default:
					ballColor = playerColorPink;
					break;
			}
			switch(bR)
			{
				case 0:
					ballRadius = playerBallRadius * 1;
					//ballRadius = playerBallRadius * 1.2;
					break;
				case 1:
					ballRadius = playerBallRadius * 0.75;
					//ballRadius = playerBallRadius * 0.9;
					break;
				case 2:
					ballRadius = playerBallRadius * 0.5;
					//ballRadius = playerBallRadius * 0.6;
					break;
				default:
					ballRadius = playerBallRadius * 0.75;
					//ballRadius = playerBallRadius * 0.9;
					break;		
			}
			
			var cirBall;
			var fd = new b2FixtureDef;
			
			var circle = new b2CircleShape;
			{	
				bd.position = new b2Vec2(xp, yp);
				bd.type = b2Body.b2_dynamicBody;
				cirBall = world.CreateBody(bd);
				cirBall.type = 'player';
				cirBall.SetGravityScale(0);
				cirBall.SetSleepingAllowed(0);
				cirBall.color = ballColor;
				fd.restitution = 1;
				circle.m_radius = ballRadius;
				fd.shape = circle;
				fd.density = 0.1;
				//if(circle.m_radius != playerBallRadius * 0.5)
				//{
					fd.filter.categoryBits = NBALL;
				//	fd.filter.groupIndex = -1;
				//}
				//else
				//{
				//	fd.filter.categoryBits = SBALL;
				//	fd.filter.maskBits = SBALL;
				//}
				cirBall.CreateFixture(fd);
			}
			cirList.push(cirBall);
		}
	}	
	
	bd.type = b2Body.b2_staticBody;
    bd.position = new b2Vec2(0.0, 0.0);
    var boundary = world.CreateBody(bd);

    var distKX = 1;
    var distKY = 4;
    var boxSize = 1.6*playerBallRadius;
    // upper boundary
	{
		var fd = new b2FixtureDef;
        var poly = new b2PolygonShape;
        poly.SetAsBox(window.innerWidth/2, playerBallRadius, new b2Vec2(0, window.innerHeight/SCALE + playerBallRadius), 0);
        fd.shape = poly;
        boundary.CreateFixture(fd);
    }
    // lower boundary
	{
		var fd = new b2FixtureDef;
        var poly = new b2PolygonShape;
        poly.SetAsBox(window.innerWidth/2, playerBallRadius, new b2Vec2(0, -playerBallRadius), 0);
        fd.shape = poly;
        boundary.CreateFixture(fd);
    }
	// left boundary
	{
		var fd = new b2FixtureDef;
        var poly = new b2PolygonShape;
        poly.SetAsBox(playerBallRadius, window.innerHeight/2, new b2Vec2(window.innerWidth/(2*SCALE) + playerBallRadius, window.innerHeight/2), 0);
        fd.shape = poly;
        boundary.CreateFixture(fd);
    }
	// right boundary
	{
		var fd = new b2FixtureDef;
        var poly = new b2PolygonShape;
        poly.SetAsBox(playerBallRadius, window.innerHeight/2, new b2Vec2(- window.innerWidth/(2*SCALE) - playerBallRadius, window.innerHeight/2), 0);
        fd.shape = poly;
        boundary.CreateFixture(fd);
    }
}


function draw() 
{
	if (!isPaused)
	{
		var tStart = new Date().valueOf();
		world.Step(
			1 / 60,		//frame-rate
			8, 		//velocity iterations
			3			//position iterations
		);
		var tEnd = new Date().valueOf();
		msAccumulator += (tEnd - tStart);
		if (frameCount == SamplePeriod) {
            var simTime = Math.round(msAccumulator * 10 / frameCount) / 10;
		    frameCount = 0;
		    msAccumulator = 0;
		    document.getElementById("sim").innerHTML = "Sim:" + simTime + "ms";
		} else {
			frameCount++;
		}

		fancyDraw.Clear();
		fancyDraw.Draw(world);

		world.ClearForces();
		//checkMode();
		pushPull();
	}
	else
		play();
	requestAnimFrame(draw);
}

function pushPull()
{
	var	b2Vec2 = Box2D.Common.Math.b2Vec2;
	var cX, cY;
	var dt;
	var r = touchSize;

	for(var j = 0; j < cirList.length; j++)
	{
		cX = cirList[j].GetWorldCenter().x;
		cY = cirList[j].GetWorldCenter().y;
		dt = Math.sqrt((tX - cX) * (tX - cX) + (tY - cY) * (tY - cY));
		/*if(dt < r)
		{
			var k = 20 * (1 - dt/r) * fDir;
			var fX = k*(tX - cX);
			var fY = k*(tY - cY);
			cirList[j].ApplyForce(new b2Vec2(fX, fY), new b2Vec2(cX, cY));
		}
		else*/
		{
			var k = 5;
			var fX = k*(cirCordX[j] - cX);
			var fY = k*(cirCordY[j] - cY);
			var fX1 = k*(arrListX[j] - cX);
			var fY1 = k*(cirCordY[j] - cY);
			if(initArrange == 1)
			{
				if(cX != cirCordX[j] && cY !=  cirCordY[j])
					cirList[j].SetLinearVelocity(new b2Vec2(fX, fY));
				else	
					cirList[j].SetLinearVelocity(new b2Vec2(0, 0));
			}
			else if(initArrange == 2)
			{
				if(cX != arrListX[j] && cY !=  arrListY[j])
					cirList[j].SetLinearVelocity(new b2Vec2(fX1, fY1));
				else	
					cirList[j].SetLinearVelocity(new b2Vec2(0, 0));
			}	
		}
	}
}

function checkMode()
{
	var	b2Vec2 = Box2D.Common.Math.b2Vec2;
	//var e = document.getElementById("mode");
	var modeVal = 1;//e.options[e.selectedIndex].value;
	
	
	if(modeVal != fDir)
		fDir = modeVal;
	
	//var f = document.getElementById("number");
	var numberVal = 100;//f.options[f.selectedIndex].value;
	if(numberVal != numOfCir)
	{
		if(numberVal == 200)
			playerBallRadius = 0.6 * (w+h)/50;
		else if(numberVal == 300)
			playerBallRadius = 0.5 * (w+h)/50;
		else if(numberVal == 500)
			playerBallRadius = 0.4 * (w+h)/50;	
		else if(numberVal == 1000)
			playerBallRadius = 0.3 * (w+h)/50;
		else if(numberVal == 2000)	
			playerBallRadius = (w+h)/50 * 0.2;
		else if(numberVal == 3000)
			playerBallRadius = 0.1 * (w+h)/50;
		else
			playerBallRadius = 0.8 * (w+h)/50;
		
		numOfCir = numberVal;
		for(var i = 0; i < cirList.length; i++)
		{
			world.DestroyBody(cirList[i]);
		}
		world.DestroyBody(touchBall);
		cirList = [];
		cirCordX = [];
		cirCordY = [];
		touchPlay();
	}
	
	//var g = document.getElementById("force");
	var forceVal = 1;//g.options[g.selectedIndex].value;
	//alert(forceVal);
	if(forceVal != initForceVal)
	{
		touchSize = ((w + h)/8) * forceVal * 0.4;
		initForceVal = forceVal;
		for(var i = 0; i < cirList.length; i++)
		{
			world.DestroyBody(cirList[i]);
		}
		cirList = [];
		cirCordX = [];
		cirCordY = [];
		world.DestroyBody(touchBall);
		touchPlay();
	}
	
	/*var ht = document.getElementById("arrange");
	var arrangeVal = ht.options[ht.selectedIndex].value;
	//console.log(arrangeVal + "init" + initArrange);
	if(arrangeVal != initArrange)
	{
		initArrange = arrangeVal;
		var xnew = 0, ynew = h/6;
		var cirRad = h/3;
		var nBall = cirList.length;
		var plusOrMinus;
		var yS = (2*h/3)/nBall;
		switch(arrangeVal)
		{
			case "1":
				for(var i = 0; i < cirList.length; i++)
				{
					world.DestroyBody(cirList[i]);
				}
				cirList = [];
				cirCordX = [];
				cirCordY = [];
				world.DestroyBody(touchBall);
				touchPlay();
				break;
			case "2":
				for(var i = 1; i <= nBall; i++)
				{
					plusOrMinus = Math.random() < 0.5 ? -1 : 1;
					ynew = (h/6) + i * yS;
					xnew = (plusOrMinus * Math.sqrt( cirRad * cirRad - (ynew - h/2)*(ynew - h/2)));
					arrListX.push(xnew);
					arrListY.push(ynew);
				}
				for(var j=0; j < nBall; j++)
				{
					var fX = (arrListX[j] - cirCordX[j]);
					var fY = (arrListY[j] - cirCordY[j]);
					if(arrListX[j] != cirCordX[j] && arrListY[j] !=  cirCordY[j])
						cirList[j].SetLinearVelocity(new b2Vec2(fX, fY));
					else	
						cirList[j].SetLinearVelocity(new b2Vec2(0, 0));
				}
				break;
		}
	} */
//console.log(world.GetBodyCount ());
}




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
 
function FancyDraw() {

    this.m_ctx = null;
    this.m_xform = null;
    this.m_width = 0;
    this.m_height = 0;
    
    this.SetCanvas = function(canvas) {
    	var element = document.getElementById(canvas);
    	this.m_width = element.width;
		this.m_height = element.height;
    	this.m_ctx = element.getContext("2d");
    	
    	// canvas element has no getTransform(), so need to track ourselves...
    	this.m_xform = new Transform();
    };
    
    this.Clear = function() {   	
		this.clearCanvasTransform();
    	this.m_ctx.clearRect (0 , 0 , this.m_width, this.m_height);    	
  		this.setCanvasTransform(); 
    };
    
    this.Scale = function(scale) {
    	this.m_xform.scale(scale, scale);
  		this.setCanvasTransform(); 
    };
    
    this.Translate = function(x, y) {
    	this.m_xform.translate(x, y);
  		this.setCanvasTransform(); 
    };
    
  	this.Rotate = function(theta) {
  		this.m_xform.rotate(theta);
  		this.setCanvasTransform(); 		
    };
    
    this.DrawCircle = function(center, radius, isAwake, color) {
		if(isAwake)
		{	
			if(typeof(color) == "undefined")
			this.m_ctx.fillStyle = 'rgba(255, 248, 220, 1)';
			else
			this.m_ctx.fillStyle = color;
		}
    	else
    		this.m_ctx.fillStyle = 'rgba(128, 128, 128, 1)';
    		
        this.m_ctx.beginPath();
	    this.m_ctx.arc(center.x, center.y, radius, 0, Math.PI*2, true);
	    this.m_ctx.closePath();
        this.m_ctx.fill(); 
    };
    
    this.clearCanvasTransform = function() {
    	// m11, m12, m21, m22, dx, dy
    	this.m_ctx.setTransform(1, 0, 0, 1, 0, 0);
    };
    
    this.setCanvasTransform = function() {
  		var m = this.m_xform.m;
		this.m_ctx.setTransform(m[0], m[1], m[2], m[3], m[4], m[5]);
	};
    
	this.DrawPolygon = function(transform, vertices, vertexCount, isAwake, color) { // Added color as last argument
		if(isAwake)
		{	
			if(typeof(color) == "undefined")
			this.m_ctx.fillStyle = 'rgba(255, 0, 0, 1)'; //Red by default
			else
			this.m_ctx.fillStyle = color;
		}
    	else
    		this.m_ctx.fillStyle = 'rgba(128, 128, 128, 1)';
    	
    	// sg: transform each vertex to world coordinates
    	//
    	var v = Box2D.Common.Math.b2Math.Mul(transform, vertices[0]);

		this.m_ctx.beginPath();
		this.m_ctx.moveTo(v.x, v.y);
		for (var i = 1; i < vertexCount; i++) {
		    v = Box2D.Common.Math.b2Math.Mul(transform, vertices[i]);
			this.m_ctx.lineTo(v.x, v.y);
		}
		this.m_ctx.closePath();
        this.m_ctx.fill();         
    };
	
	this.DrawEdge = function(transform, vertices, isAwake, color) { // For b2EdgeShape
		if(isAwake)
		{	
			if(typeof(color) == "undefined")
			this.m_ctx.fillStyle = 'rgba(255, 0, 0, 1)'; //Red by default
			else
			this.m_ctx.fillStyle = color;
		}
    	else
    		this.m_ctx.fillStyle = 'rgba(128, 128, 128, 1)';
    	
    	// sg: transform each vertex to world coordinates
    	//
    	var v = Box2D.Common.Math.b2Math.Mul(transform, vertices[0]);
    	
		this.m_ctx.beginPath();
		this.m_ctx.moveTo(x, y);
		
		v = Box2D.Common.Math.b2Math.Mul(transform, vertices[1]);
		
		this.m_ctx.lineTo(x, y);   
		this.m_ctx.strokeStyle = color;
		this.m_ctx.stroke();
    };
	
   	this.Draw = function(aWorld) {;
   		var b2Shape = Box2D.Collision.Shapes.b2Shape;
   		var bodyList = aWorld.GetBodyList();
		for (var body = bodyList; body; body = body.GetNext()) {
			var fixtureList = body.GetFixtureList();
			for(var fixture = fixtureList; fixture; fixture = fixture.GetNext()) {
				var shape = fixture.GetShape();
				switch(shape.GetType()) {
				case b2Shape.e_circle:
					this.DrawCircle(body.GetPosition(), shape.m_radius, body.IsAwake(), body.color);
					break;
				case b2Shape.e_polygon:
					this.DrawPolygon(body.GetTransform(), shape.m_vertices, shape.GetVertexCount(), body.IsAwake(), body.color);
					break;
				case b2Shape.e_edge:
					var vertices =  new Array();
					vertices[0] = shape.m_vertex1;
					vertices[1] = shape.m_vertex2;
					this.DrawEdge(body.GetTransform(), vertices, 2, body.IsAwake(), body.color);
					break;
				}
			}
		}
    };	
}

var DEGTORAD = 0.0174532925;

function newStream() //Function that creates new stream
{
	//simSampler.startFrame();
	world.Step(
		1 / 60,		//frame-rate
		10, 		//velocity iterations
		10			//position iterations
	);
	
	//simSampler.endFrame();
	if (n_count < e_count)
	{
		b2BodyDef = Box2D.Dynamics.b2BodyDef,
		b2Body = Box2D.Dynamics.b2Body,
		b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape,
		b2Vec2 = Box2D.Common.Math.b2Vec2,
		b2World = Box2D.Dynamics.b2World;

        var bd = new b2BodyDef;
		var shape = new b2PolygonShape;
		bd.type = b2Body.b2_dynamicBody;

		bd.position.Set(globalX, e_height);
		var body = world.CreateBody(bd);
		
		if( globalX < -11)								// Assign different colors for different area (Left, Center & Right) 
		body.color = 'rgba(0, 255, 0, 1)';
		else if( globalX >= -11 && globalX < 11)
		body.color = 'rgba(64, 64, 255, 1)';
		else if( globalX >= 11)
		body.color = 'rgba(255, 0, 0, 1)';
		shape.SetAsBox(0.125, 0.125);
		body.CreateFixture2(shape, 1.0);
		++n_count;
	}
	//fancyDraw.Clear();
	//fancyDraw.Draw(world);
		
	//world.ClearForces();

    //fpsSampler.markFrame();
    requestAnimFrame(newStream);
}

// Variables Initialiation

var e_count = 100;  						// Particle count of each stream
var no_of_streams = 3; 						// Number of streams
var streamCount = e_count * no_of_streams;  // Initial total number of particles inside the boundary
var explosionCount = 50.0;					// No. of particles generated per explosion

generateParticles = function(x, y)
{
	b2BodyDef = Box2D.Dynamics.b2BodyDef,
	b2FixtureDef = Box2D.Dynamics.b2FixtureDef,
	b2Body = Box2D.Dynamics.b2Body,
	b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape,
	b2Vec2 = Box2D.Common.Math.b2Vec2,
	b2World = Box2D.Dynamics.b2World;
	
	var scale = 0.2;	
	if(navigator.userAgent.toLowerCase().indexOf('firefox') > -1) // Check if browser is firefox
		scale = 0.1;
	var X = -x*scale+36;	// Mouse co-ordinates to Box2D boundary co-ordinates
	var Y = 87.5-y*scale;
	if( Y < 0 || Y > 67.3 || X > 33 || X < -33) // Check if mouse input co-ordinate is outside Box2D boundary
	return;
	else if( Y < 62 && Y > 58)		// Check if mouse input co-ordinate is inside stream source bar
	{	
		globalX = X;
		n_count = 0;
		requestAnimFrame(newStream);
		this.par = document.getElementById("particles"); //Pass variable to html to print out
		streamCount = streamCount + e_count;
		this.par.firstChild.nodeValue = streamCount;
		return;
	}

	var blastPower = 50, c, angle, rayDir;
	var polygonShape = new b2PolygonShape;
    polygonShape.SetAsBox(0.125, 0.125);
    var fd = new b2FixtureDef;
    fd.shape = polygonShape;
    fd.restitution = 1.0;
    fd.filter.groupIndex = -1;
    var bd = new b2BodyDef;
    bd.type = b2Body.b2_dynamicBody;
	bd.position.Set(X, Y);
	var body = new Array();
    for (var i = 0; i < explosionCount; i++)
    {
     	angle = (i / explosionCount) * 360 * DEGTORAD;
     	rayDir = new b2Vec2(Math.sin(angle), Math.cos(angle));
     	rayDir.x = rayDir.x * blastPower;
     	rayDir.y = rayDir.y * blastPower;
      	bd.linearVelocity = rayDir;
      	body[i] = world.CreateBody(bd);
      	c = Math.floor((Math.random()*100)) % 3;
		switch(c) //Switch case to generate random colors(R, G, B) during explosion
		{
			case 0: body[i].color = 'rgba(255, 0, 0, 1)';
					break;
			case 1: body[i].color = 'rgba(0, 255, 0, 1)';
					break;
			case 2: body[i].color = 'rgba(64, 64, 255, 1)';
		}
      	body[i].CreateFixture(fd);
    }
	this.par = document.getElementById("particles");
	streamCount = streamCount + explosionCount;
	this.par.firstChild.nodeValue = streamCount;
};

var fancyDraw = new FancyDraw();
    
var SAMPLECOUNT = 60;  // calculate fps over this many frames
var fpsSampler = new FpsSampler(SAMPLECOUNT, "fps");
var simSampler = new MSecSampler(SAMPLECOUNT, "sim");

var SCALE = 10;
var world = null;

// particle count is m x n
var m_count = null;
var n_count = 0;
var e_height = null;
var	e_columnCount = null;
var	e_rowCount = null;

function init() {
     if (window.testRunner || window.layoutTestController) {
         testRunner.waitUntilDone();
         testRunner.dumpAsText();
     }
    if(typeof(WebBox2D) === "undefined") {
        document.getElementById("b2j").style.visibility = "visible"; 
        document.getElementById("b2n").style.visibility = "hidden";
        Box2DWebLoader(initPage);
    }
    else {
        document.getElementById("b2j").style.visibility = "hidden"; 
        document.getElementById("b2n").style.visibility = "visible";
        Box2D = WebBox2D;
        initPage();
    }
    
    // set scale for Tizen / PC
    /* if(navigator.platform.indexOf("arm") !== -1)
        document.body.style.zoom = 1.35;
    else */
        document.body.style.zoom = 0.5;
}

function initPage() 
{
	var	b2Vec2 = Box2D.Common.Math.b2Vec2,
		b2BodyDef = Box2D.Dynamics.b2BodyDef,
		b2Body = Box2D.Dynamics.b2Body,
		b2FixtureDef = Box2D.Dynamics.b2FixtureDef,
		b2Fixture = Box2D.Dynamics.b2Fixture,
		b2RevoluteJointDef = Box2D.Dynamics.Joints.b2RevoluteJointDef,
		b2World = Box2D.Dynamics.b2World,
		b2MassData = Box2D.Collision.Shapes.b2MassData,
		b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape,
		b2CircleShape = Box2D.Collision.Shapes.b2CircleShape,
		b2EdgeShape = Box2D.Collision.Shapes.b2EdgeShape;

	world = new b2World(
	   	new b2Vec2(0, -10),	//gravity
		true				//allow sleep
	);
	
	// Box2D Testbed code here...
		var count = 0;
		this.par = document.getElementById("particles");  //Initial particle count
		this.par.firstChild.nodeValue = e_count * no_of_streams;
		e_height = 60;
		e_columnCount = 500;
		e_rowCount = 100;
		{
			var brick_size = 0.2;
			//Barriers
			{
				var bd = new b2BodyDef;
				bd.type = b2Body.b2_staticBody;
				bd.allowSleep = false;
				var body = world.CreateBody(bd);
				body.color = 'rgba(255, 248, 220, 1)';
				var fd = new b2FixtureDef;
				fd.restitution = 1;
				var shape = new b2PolygonShape;
                var angle = 45;
				
				//Upside Barrier
				
				shape.SetAsOrientedBox(0.2, 4, new b2Vec2( 20.0, 45.0), (angle+90) * DEGTORAD);   
                fd.shape = shape;
                body.CreateFixture(fd);
				shape.SetAsOrientedBox(0.2, 4, new b2Vec2( -20.0, 45.0), angle * DEGTORAD);
                fd.shape = shape;
                body.CreateFixture(fd);
				
				//Downside Barrier
				shape.SetAsOrientedBox(0.2, 5, new b2Vec2( 16.0, 20.0), (angle+90) * DEGTORAD );
                fd.shape = shape;
                body.CreateFixture(fd);
				shape.SetAsOrientedBox(0.2, 5, new b2Vec2( -16.0, 20.0), angle * DEGTORAD );
                fd.shape = shape;
                body.CreateFixture(fd);
				
				//Center Barrier
				shape.SetAsOrientedBox(0.2, 5, new b2Vec2( -8, 33.0), (angle+90) * DEGTORAD );
                fd.shape = shape;
                body.CreateFixture(fd);				
				shape.SetAsOrientedBox(0.2, 5, new b2Vec2( 8, 33.0), angle * DEGTORAD);
                fd.shape = shape;
                body.CreateFixture(fd);

				//Center Barrier bridge
				shape.SetAsOrientedBox(0.2, 2, new b2Vec2( 1.8, 26.0), (angle+25) * DEGTORAD);
                fd.shape = shape;
                body.CreateFixture(fd);
                shape.SetAsOrientedBox(0.2, 2, new b2Vec2( -1.8, 26.0), (angle+65) * DEGTORAD);
                fd.shape = shape;
                body.CreateFixture(fd);
			}
			//Ground
			var bd = new b2BodyDef;
			{
				bd.position.y = -brick_size;
				var ground = world.CreateBody(bd);
				ground.color = 'rgba(255, 248, 220, 1)';
				var N = 175;
				var M = 5;
				var position = new b2Vec2;
				position.y = 0.0;
				for (var j = 0; j < M; ++j)
				{
					position.x = -N * brick_size;
					for (var i = 0; i < N; ++i)
					{
						var shape = new b2PolygonShape;
						shape.SetAsOrientedBox(brick_size, brick_size, position, 0.0);
                        var fd = new b2FixtureDef;
                        fd.shape = shape;
                        fd.restitution = 1;
                        ground.CreateFixture(fd);
						position.x += 2.0 * brick_size;
					}
					position.y -= 2.0 * brick_size;
				}
			}
			//Left Wall
			{
				bd.position.x = -brick_size;
				bd.position.y = 34;
				var ground = world.CreateBody(bd);
				ground.color = 'rgba(255, 248, 220, 1)';
				var N = 170;
				var M = 4;
				var position = new b2Vec2;
				position.x = 34.8;
				for (var j = 0; j < M; ++j)
				{
					position.y = -N * brick_size;
					for (var i = 0; i < N; ++i)
					{
						var shape = new b2PolygonShape;
						shape.SetAsOrientedBox(brick_size, brick_size, position, 0.0);
                        var fd = new b2FixtureDef;
                        fd.shape = shape;
                        fd.restitution = 1;
                        ground.CreateFixture(fd);
						position.y += 2.0 * brick_size;
					}
					position.x -= 2.0 * brick_size;
				}
			}
			//Right Wall
			{
				bd.position.x = -brick_size;
				bd.position.y = 34.0;
				var ground = world.CreateBody(bd);
				ground.color = 'rgba(255, 248 220, 1)';
				var N = 170;
				var M = 4;
				var position = new b2Vec2;
				position.x = -33.6;
				for (var j = 0; j < M; ++j)
				{
					position.y = -N * brick_size;
					for (var i = 0; i < N; ++i)
					{
						var shape = new b2PolygonShape;
						shape.SetAsOrientedBox(brick_size, brick_size, position, 0.0);
                        var fd = new b2FixtureDef;
                        fd.shape = shape;
                        fd.restitution = 1;
                        ground.CreateFixture(fd);
						position.y += 2.0 * brick_size;
					}
					position.x -= 2.0 * brick_size;
				}
			}
            //Upper Wall
			{
				bd.position.y = -brick_size;
				bd.position.x = 0.0;
				var ground = world.CreateBody(bd);
				ground.color = 'rgba(255, 248, 220, 1)';
				var N = 175;
				var M = 5;
				var position = new b2Vec2;
				position.y = 69.8;
				for (var j = 0; j < M; ++j)
				{
					position.x = -N * brick_size;
					for (var i = 0; i < N; ++i)
					{
						var shape = new b2PolygonShape;
						shape.SetAsOrientedBox(brick_size, brick_size, position, 0.0);
                        var fd = new b2FixtureDef;
                        fd.shape = shape;
                        fd.restitution = 1;
                        ground.CreateFixture(fd);
						position.x += 2.0 * brick_size;
					}
					position.y -= 2.0 * brick_size;
				}
			}
			m_count = 0;
		}
	
	document.addEventListener('mousedown', function(event) 
	{
			var s = 1; 
			if(navigator.userAgent.toLowerCase().indexOf('firefox') > -1)
				 s = 2; 		//Scale for Firefox
			if((event.clientY < (s*127.5)) || (event.clientY > (s*147.5)))
				generateParticles(event.clientX, event.clientY);
			else if(n_count == e_count || n_count === 0)  			//If current stream is done, then generate new stream
			{
				if(++count <= 20)
				generateParticles(event.clientX, event.clientY);
			}
	});
	
	var w = document.getElementById("canvas").width;
	var h = document.getElementById("canvas").height;
	
	fancyDraw.SetCanvas("canvas");
	
	// move origin down and invert y (to match coords used in C++ Testbed)
	fancyDraw.Translate(w/2, (8*h)/10);
	fancyDraw.Rotate(Math.PI);
	fancyDraw.Scale(SCALE);

	requestAnimFrame(draw);
}

function step() {

}

function draw() {
	simSampler.startFrame();
	world.Step(
		1 / 60,		//frame-rate
		10, 		//velocity iterations
		10			//position iterations
	);
	
	simSampler.endFrame();
	
	if (m_count < e_count)
	{
		b2BodyDef = Box2D.Dynamics.b2BodyDef,
		b2Body = Box2D.Dynamics.b2Body,
		b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape,
		b2Vec2 = Box2D.Common.Math.b2Vec2,
		b2World = Box2D.Dynamics.b2World;

        var bd = new b2BodyDef;
		var shape = new b2PolygonShape;
		bd.type = b2Body.b2_dynamicBody;
	
		bd.position.Set(20.0, e_height);
		var body1 = world.CreateBody(bd);
		body1.color = 'rgba(255, 0, 0, 1)';
		shape.SetAsBox(0.125, 0.125);
		body1.CreateFixture2(shape, 1.0);

		bd.position.Set(0.0, e_height);
		var body2 = world.CreateBody(bd);
        body2.color = 'rgba(64, 64, 255, 1)';
		shape.SetAsBox(0.125, 0.125);
		body2.CreateFixture2(shape, 1.0);

		bd.position.Set(-20.0, e_height);
		var body3 = world.CreateBody(bd);
		body3.color = 'rgba(0, 255, 0, 1)';
		shape.SetAsBox(0.125, 0.125);
		body3.CreateFixture2(shape, 1.0);
		++m_count;
	}

	fancyDraw.Clear();
	fancyDraw.Draw(world);
		
	world.ClearForces();

    fpsSampler.markFrame();
    requestAnimFrame(draw); 
}
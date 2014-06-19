/*
*
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Copyright (c) 2014, Samsung Electronics Co. Ltd.*/



function FancyDraw() {

    this.m_ctx = null;
    this.m_xform = null;
    this.m_width = 0;
    this.m_height = 0;
 
    this.b2Shape;
    this.b2Math;
    	
    this.SetCanvas = function(canvas) {
    	var element = document.getElementById(canvas);
		element.width = window.innerWidth;
		element.height = window.innerHeight;
		this.m_width = element.width;
		this.m_height = element.height;
		this.m_ctx = element.getContext("2d");
    	
    	// canvas element has no getTransform(), so need to track ourselves...
    	this.m_xform = new Transform();
    };
    
    this.Clear = function() {   	
		this.clearCanvasTransform();
		this.m_ctx.fillStyle = "rgba(30, 29, 30, 1)";
    	this.m_ctx.fillRect (0 , 0 , this.m_width, this.m_height);    
    	//this.m_ctx.clearRect (0 , 0 , this.m_width, this.m_height);    
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
    
    this.DrawCircle = function(center, radius, isAwake, color, userData) {
        if (userData) {
        	
        	this.m_ctx.drawImage(userData,0,0);
        	return;
        }
		if(typeof(color) == "undefined")
			this.m_ctx.fillStyle = 'rgba(255, 248, 220, 1)';
		else
			this.m_ctx.fillStyle = color;
    	
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

		if(typeof(color) == "undefined")
			this.m_ctx.fillStyle = 'rgba(255, 0, 0, 1)'; //Red by default
		else
			this.m_ctx.fillStyle = color;
    	
    	// sg: transform each vertex to world coordinates

    	var v = this.b2Math.Mul(transform, vertices[0]);

		this.m_ctx.beginPath();
		this.m_ctx.moveTo(v.x, v.y);
		for (var i = 1; i < vertexCount; i++) {
		    v = this.b2Math.Mul(transform, vertices[i]);
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
    	
    	var v = Box2D.Common.Math.b2Math.Mul(transform, vertices[0]);
    	
		this.m_ctx.beginPath();
		this.m_ctx.moveTo(x, y);
		
		v = Box2D.Common.Math.b2Math.Mul(transform, vertices[1]);
		
		this.m_ctx.lineTo(x, y); 
		this.m_ctx.strokeStyle = color;
		this.m_ctx.stroke();
    };
	
   	this.Draw = function(aWorld) {
   		//var b2Shape = Box2D.Collision.Shapes.b2Shape;
   		var bodyList = aWorld.GetBodyList();
		for (var body = bodyList; body; body = body.GetNext()) {
			var fixtureList = body.GetFixtureList();
			for(var fixture = fixtureList; fixture; fixture = fixture.GetNext()) {
				var shape = fixture.GetShape();
				switch(shape.GetType()) {
				case this.b2Shape.e_circle:
					this.DrawCircle(body.GetPosition(), shape.m_radius, body.IsAwake(), body.color, body.GetUserData());
					break;
				case this.b2Shape.e_polygon:
					this.DrawPolygon(body.GetTransform(), shape.m_vertices, shape.GetVertexCount(), body.IsAwake(), body.color);
					break;
				}
			}
		}
    };
}
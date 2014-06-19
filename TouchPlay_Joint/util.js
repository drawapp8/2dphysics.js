/*
*
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Copyright (c) 2014, Samsung Electronics Co. Ltd.*/

function FpsSampler(aSampleCount, aDivId) {

    this.sampleCount = aSampleCount;
    this.fpsDivId = aDivId;
    this.fpsDiv = null;
    this.fps = 0;
    this.frameCount = 0;
    this.tStart = null;
    
    this.markFrame = function() {
    	if(this.frameCount == 0) {
			this.tStart = new Date().valueOf();
        }

        if(this.frameCount === this.sampleCount) {
            var tNow = new Date().valueOf();    
            var delta = Math.max(1, tNow - this.tStart);
            this.fps = Math.round((this.sampleCount * 1000) / delta);
            this.frameCount = 0;
            this.display();
        }
        else {
            this.frameCount++;
        }
    };
       
    this.display = function() {
    	if(this.fpsDiv === null) this.fpsDiv = document.getElementById(this.fpsDivId);
    	this.fpsDiv.firstChild.nodeValue = this.fps;
    };
}

function MSecSampler(aSamplePeriod, aDivId) {

    this.samplePeriod = aSamplePeriod;
    this.msDivId = aDivId;
    this.msDiv = null;
    this.ms = 0;
    this.msAccumulator = 0;
    this.frameCount = 0;
    this.tStart = null;
    this.isAccumulating = false;    // allow calling endFrame before startFrame
    
    this.startFrame = function() {
        if(this.isAccumulating) return;
        this.isAccumulating = true;
        
    	if(this.frameCount % this.samplePeriod == 0) {
            this.msAccumulator = 0;
    		this.frameCount = 0;
    		this.display();
    	}
		this.tStart = new Date().valueOf();
    };
    
    this.endFrame = function() {
        if(!this.isAccumulating) return;
        this.isAccumulating = false;
        
    	var tNow = new Date().valueOf();	
        this.msAccumulator += (tNow - this.tStart);
		this.frameCount++;
        if(this.frameCount % this.samplePeriod == 0) {
            this.ms = Math.round(10 * this.msAccumulator / this.frameCount) / 10;
            this.frameCount = 0;
        }
    };
    
    this.display = function() {
    	if(this.msDiv === null) this.msDiv = document.getElementById(this.msDivId);
    	this.msDiv.firstChild.nodeValue = this.ms + " ms";
    };
}

// dynamically load Box2dWeb.js
//
function Box2DWebLoader(initPageCB)
{
    var head = document.getElementsByTagName('head')[0];
    var script= document.createElement('script');
    script.type= 'text/javascript';

    script.onload = function() {
        initPageCB();
    }; 
   
    script.src= '../Box2dWeb-2.2.1/Box2dWeb-2.2.1.js';
    head.appendChild(script);
}

function Box2DDebugDrawLoader(path, initPageCB)
{
    var head = document.getElementsByTagName('head')[0];
    var script= document.createElement('script');
    script.type= 'text/javascript';

    script.onload = function() {
        initPageCB();
    }; 
   
    script.src= path + 'b2DebugDraw.js';
    head.appendChild(script);
}


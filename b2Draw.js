
/*
 * JS implementation of b2Draw
 * (excerpted from Box2dWeb-2.1.a.3.js)
 */

var b2Draw = Box2D.Dynamics.b2Draw;

/*
b2Draw = function () {
  this.m_drawScale = 1.0;
  this.m_lineThickness = 1.0;
  this.m_alpha = 1.0;
  this.m_fillAlpha = 1.0;
  this.m_xformScale = 1.0;
  var __this = this;
  //#WORKAROUND
  this.m_sprite = {
	 graphics: {
		clear: function () {
		   __this.m_ctx.clearRect(0, 0, __this.m_ctx.canvas.width, __this.m_ctx.canvas.height)
		}
	 }
  };
};

b2Draw.e_shapeBit = 0x0001;
b2Draw.e_jointBit = 0x0002;
b2Draw.e_aabbBit = 0x0004;
b2Draw.e_pairBit = 0x0008;
b2Draw.e_centerOfMassBit = 0x0010;
b2Draw.e_controllerBit = 0x0020;
*/

b2Draw.prototype.Clear = function () {
    this.m_ctx.clearRect(0, 0, this.m_ctx.canvas.width, this.m_ctx.canvas.height);
}

b2Draw.prototype._color = function (color, alpha) {
  return "rgba(" + ((color & 0xFF0000) >> 16) + "," + ((color & 0xFF00) >> 8) + "," + (color & 0xFF) + "," + alpha + ")";
};

/*
b2Draw.prototype.b2Draw = function () {
  this.m_drawFlags = 0;
};

b2Draw.prototype.SetFlags = function (flags) {
  if (flags === undefined) flags = 0;
  this.m_drawFlags = flags;
};

b2Draw.prototype.GetFlags = function () {
  return this.m_drawFlags;
};

b2Draw.prototype.AppendFlags = function (flags) {
  if (flags === undefined) flags = 0;
  this.m_drawFlags |= flags;
};

b2Draw.prototype.ClearFlags = function (flags) {
  if (flags === undefined) flags = 0;
  this.m_drawFlags &= ~flags;
};
*/

b2Draw.prototype.SetSprite = function (sprite) {
  this.m_ctx = sprite;
  if (this.m_drawScale == undefined)
    this.m_drawScale = 1.0;
  if (this.m_lineThickness == undefined)
    this.m_lineThickness = 1.0;
  if (this.m_alpha == undefined)
    this.m_alpha = 1.0;
  if (this.m_fillAlpha == undefined)
    this.m_fillAlpha = 1.0;
  if (this.m_xformScale == undefined)
    this.m_xformScale = 1.0;
};

b2Draw.prototype.GetSprite = function () {
  return this.m_ctx;
};

b2Draw.prototype.SetDrawScale = function (drawScale) {
  if (drawScale === undefined) drawScale = 0;
  this.m_drawScale = drawScale;
};
b2Draw.prototype.GetDrawScale = function () {
  return this.m_drawScale;
};

b2Draw.prototype.SetLineThickness = function (lineThickness) {
  if (lineThickness === undefined) lineThickness = 0;
  this.m_lineThickness = lineThickness;
  this.m_ctx.strokeWidth = lineThickness;
};

b2Draw.prototype.GetLineThickness = function () {
  return this.m_lineThickness;
};

b2Draw.prototype.SetAlpha = function (alpha) {
  if (alpha === undefined) alpha = 0;
  this.m_alpha = alpha;
};

b2Draw.prototype.GetAlpha = function () {
  return this.m_alpha;
};

b2Draw.prototype.SetFillAlpha = function (alpha) {
  if (alpha === undefined) alpha = 0;
  this.m_fillAlpha = alpha;
};
b2Draw.prototype.GetFillAlpha = function () {
  return this.m_fillAlpha;
};

b2Draw.prototype.SetXFormScale = function (xformScale) {
  if (xformScale === undefined) xformScale = 0;
  this.m_xformScale = xformScale;
};

b2Draw.prototype.GetXFormScale = function () {
  return this.m_xformScale;
};

b2Draw.prototype.DrawPolygon = function (vertices, vertexCount, color) {
  if (!vertexCount) return;
  var s = this.m_ctx;
  var drawScale = this.m_drawScale;
  s.beginPath();
  s.strokeStyle = this._color(color.color, this.m_alpha);
  s.moveTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
  for (var i = 1; i < vertexCount; i++) {
	 s.lineTo(vertices[i].x * drawScale, vertices[i].y * drawScale);
  }
  s.lineTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
  s.closePath();
  s.stroke();
};

b2Draw.prototype.DrawSolidPolygon = function (vertices, vertexCount, color) {
  if (!vertexCount) return;
  var s = this.m_ctx;
  var drawScale = this.m_drawScale;
  s.beginPath();
  s.strokeStyle = this._color(color.color, this.m_alpha);
  s.fillStyle = this._color(color.color, this.m_fillAlpha);
  s.moveTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
  for (var i = 1; i < vertexCount; i++) {
	 s.lineTo(vertices[i].x * drawScale, vertices[i].y * drawScale);
  }
  s.lineTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
  s.closePath();
  s.fill();
  s.stroke();
};

b2Draw.prototype.DrawCircle = function (center, radius, color) {
  if (!radius) return;
  var s = this.m_ctx;
  var drawScale = this.m_drawScale;
  s.beginPath();
  s.strokeStyle = this._color(color.color, this.m_alpha);
  s.arc(center.x * drawScale, center.y * drawScale, radius * drawScale, 0, Math.PI * 2, true);
  s.closePath();
  s.stroke();
};

b2Draw.prototype.DrawSolidCircle = function (center, radius, axis, color) {
  if (!radius) return;
  var s = this.m_ctx,
	 drawScale = this.m_drawScale,
	 cx = center.x * drawScale,
	 cy = center.y * drawScale;
  s.moveTo(0, 0);
  s.beginPath();
  s.strokeStyle = this._color(color.color, this.m_alpha);
  s.fillStyle = this._color(color.color, this.m_fillAlpha);
  s.arc(cx, cy, radius * drawScale, 0, Math.PI * 2, true);
  s.moveTo(cx, cy);
  s.lineTo((center.x + axis.x * radius) * drawScale, (center.y + axis.y * radius) * drawScale);
  s.closePath();
  s.fill();
  s.stroke();
};

b2Draw.prototype.DrawSegment = function (p1, p2, color) {
  var s = this.m_ctx,
	 drawScale = this.m_drawScale;
  s.strokeStyle = this._color(color.color, this.m_alpha);
  s.beginPath();
  s.moveTo(p1.x * drawScale, p1.y * drawScale);
  s.lineTo(p2.x * drawScale, p2.y * drawScale);
  s.closePath();
  s.stroke();
};

b2Draw.prototype.DrawTransform = function (xf) {
  var s = this.m_ctx,
	 drawScale = this.m_drawScale;
  s.beginPath();
  s.strokeStyle = this._color(0xff0000, this.m_alpha);
  s.moveTo(xf.position.x * drawScale, xf.position.y * drawScale);
  s.lineTo((xf.position.x + this.m_xformScale * xf.R.col1.x) * drawScale, (xf.position.y + this.m_xformScale * xf.R.col1.y) * drawScale);

  s.strokeStyle = this._color(0xff00, this.m_alpha);
  s.moveTo(xf.position.x * drawScale, xf.position.y * drawScale);
  s.lineTo((xf.position.x + this.m_xformScale * xf.R.col2.x) * drawScale, (xf.position.y + this.m_xformScale * xf.R.col2.y) * drawScale);
  s.closePath();
  s.stroke();
};

// this code should invoked after Box2DWeb.js has loaded (if it is loaded)
//
if(typeof(Box2D.Dynamics.b2Draw) === "undefined") {
	Box2D.Dynamics.b2Draw = b2Draw;
}

/*
 * Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
 
"use strict";

var Box2D = {};

(function(a2j, undefined) {

    if (!(Object.prototype.defineProperty instanceof Function) && Object.prototype.__defineGetter__ instanceof Function && Object.prototype.__defineSetter__ instanceof Function) {
        Object.defineProperty = function(obj, p, cfg) {
            if (cfg.get instanceof Function) obj.__defineGetter__(p, cfg.get);
            if (cfg.set instanceof Function) obj.__defineSetter__(p, cfg.set);
        }
    }

    function emptyFn() {};
    a2j.inherit = function(cls, base) {
        var tmpCtr = cls;
        emptyFn.prototype = base.prototype;
        cls.prototype = new emptyFn;
        cls.prototype.constructor = tmpCtr;
    };

    a2j.generateCallback = function generateCallback(context, cb) {
        return function() {
            cb.apply(context, arguments);
        };
    };

    a2j.NVector = function NVector(length) {
        if (length === undefined) length = 0;
        var tmp = new Array(length || 0);
        for (var i = 0; i < length; ++i)
        tmp[i] = 0;
        return tmp;
    };

    a2j.is = function is(o1, o2) {
        if (o1 === null) return false;
        if ((o2 instanceof Function) && (o1 instanceof o2)) return true;
        if ((o1.constructor.__implements != undefined) && (o1.constructor.__implements[o2])) return true;
        return false;
    };

    a2j.parseUInt = function(v) {
        return Math.abs(parseInt(v));
    }

})(Box2D);

//#TODO remove assignments from global namespace
var Vector = Array;
var Vector_a2j_Number = Box2D.NVector;
//package structure
if (typeof(Box2D) === "undefined") Box2D = {};
if (typeof(Box2D.Collision) === "undefined") Box2D.Collision = {};
if (typeof(Box2D.Collision.Shapes) === "undefined") Box2D.Collision.Shapes = {};
if (typeof(Box2D.Common) === "undefined") Box2D.Common = {};
if (typeof(Box2D.Common.Math) === "undefined") Box2D.Common.Math = {};
if (typeof(Box2D.Dynamics) === "undefined") Box2D.Dynamics = {};
if (typeof(Box2D.Dynamics.Contacts) === "undefined") Box2D.Dynamics.Contacts = {};
if (typeof(Box2D.Dynamics.Joints) === "undefined") Box2D.Dynamics.Joints = {};
if (typeof(Box2D.Rope) === "undefined") Box2D.Rope = {};
//pre-definitions
(function() {
    Box2D.Collision.IBroadPhase = 'Box2D.Collision.IBroadPhase';

    function b2AABB() {
        b2AABB.b2AABB.apply(this, arguments);
    };
    Box2D.Collision.b2AABB = b2AABB;

    function b2Collision() {
        b2Collision.b2Collision.apply(this, arguments);
    };
    Box2D.Collision.b2Collision = b2Collision;

    function b2ContactID() {
        b2ContactID.b2ContactID.apply(this, arguments);
        if (this.constructor === b2ContactID) this.b2ContactID.apply(this, arguments);
    };
    Box2D.Collision.b2ContactID = b2ContactID;

    function b2Distance() {
        b2Distance.b2Distance.apply(this, arguments);
    };
    Box2D.Collision.b2Distance = b2Distance;

    function b2DistanceInput() {
        b2DistanceInput.b2DistanceInput.apply(this, arguments);
    };
    Box2D.Collision.b2DistanceInput = b2DistanceInput;

    function b2DistanceOutput() {
        b2DistanceOutput.b2DistanceOutput.apply(this, arguments);
    };
    Box2D.Collision.b2DistanceOutput = b2DistanceOutput;

    function b2DistanceProxy() {
        b2DistanceProxy.b2DistanceProxy.apply(this, arguments);
    };
    Box2D.Collision.b2DistanceProxy = b2DistanceProxy;

    function b2DynamicTree() {
        b2DynamicTree.b2DynamicTree.apply(this, arguments);
        if (this.constructor === b2DynamicTree) this.b2DynamicTree.apply(this, arguments);
    };
    Box2D.Collision.b2DynamicTree = b2DynamicTree;

    function b2BroadPhase() {
        b2BroadPhase.b2BroadPhase.apply(this, arguments);
    };
    Box2D.Collision.b2BroadPhase = b2BroadPhase;

    function b2TreeNode() {
        b2TreeNode.b2TreeNode.apply(this, arguments);
    };
    Box2D.Collision.b2TreeNode = b2TreeNode;

    function b2Pair() {
        b2Pair.b2Pair.apply(this, arguments);
    };
    Box2D.Collision.b2Pair = b2Pair;

    function b2Manifold() {
        b2Manifold.b2Manifold.apply(this, arguments);
        if (this.constructor === b2Manifold) this.b2Manifold.apply(this, arguments);
    };
    Box2D.Collision.b2Manifold = b2Manifold;

    function b2ManifoldPoint() {
        b2ManifoldPoint.b2ManifoldPoint.apply(this, arguments);
        if (this.constructor === b2ManifoldPoint) this.b2ManifoldPoint.apply(this, arguments);
    };
    Box2D.Collision.b2ManifoldPoint = b2ManifoldPoint;

    function b2RayCastInput() {
        b2RayCastInput.b2RayCastInput.apply(this, arguments);
    };
    Box2D.Collision.b2RayCastInput = b2RayCastInput;

    function b2RayCastOutput() {
        b2RayCastOutput.b2RayCastOutput.apply(this, arguments);
    };
    Box2D.Collision.b2RayCastOutput = b2RayCastOutput;

    function b2SeparationFunction() {
        b2SeparationFunction.b2SeparationFunction.apply(this, arguments);
    };
    Box2D.Collision.b2SeparationFunction = b2SeparationFunction;

    function b2Simplex() {
        b2Simplex.b2Simplex.apply(this, arguments);
        if (this.constructor === b2Simplex) this.b2Simplex.apply(this, arguments);
    };
    Box2D.Collision.b2Simplex = b2Simplex;

    function b2SimplexCache() {
        b2SimplexCache.b2SimplexCache.apply(this, arguments);
    };
    Box2D.Collision.b2SimplexCache = b2SimplexCache;

    function b2SimplexVertex() {
        b2SimplexVertex.b2SimplexVertex.apply(this, arguments);
    };
    Box2D.Collision.b2SimplexVertex = b2SimplexVertex;

    function b2TOIInput() {
        b2TOIInput.b2TOIInput.apply(this, arguments);
    };
    Box2D.Collision.b2TOIInput = b2TOIInput;

    function b2TOIOutput() {
        b2TOIOutput.b2TOIOutput.apply(this, arguments);
    };
    Box2D.Collision.b2TOIOutput = b2TOIOutput;

    function b2WorldManifold() {
        b2WorldManifold.b2WorldManifold.apply(this, arguments);
        if (this.constructor === b2WorldManifold) this.b2WorldManifold.apply(this, arguments);
    };
    Box2D.Collision.b2WorldManifold = b2WorldManifold;

    function b2ClipVertex() {
        b2ClipVertex.b2ClipVertex.apply(this, arguments);
    };
    Box2D.Collision.b2ClipVertex = b2ClipVertex;

    function b2ContactFeature() {
        b2ContactFeature.b2ContactFeature.apply(this, arguments);
    };
    Box2D.Collision.b2ContactFeature = b2ContactFeature;

    function b2CircleShape() {
        b2CircleShape.b2CircleShape.apply(this, arguments);
        if (this.constructor === b2CircleShape) this.b2CircleShape.apply(this, arguments);
    };
    Box2D.Collision.Shapes.b2CircleShape = b2CircleShape;

    function b2EdgeShape() {
        b2EdgeShape.b2EdgeShape.apply(this, arguments);
        if (this.constructor === b2EdgeShape) this.b2EdgeShape.apply(this, arguments);
    };
    Box2D.Collision.Shapes.b2EdgeShape = b2EdgeShape;

    function b2ChainShape() {
        b2ChainShape.b2ChainShape.apply(this, arguments);
        if (this.constructor === b2ChainShape) this.b2ChainShape.apply(this, arguments);
    };
    Box2D.Collision.Shapes.b2ChainShape = b2ChainShape;

    function b2MassData() {
        b2MassData.b2MassData.apply(this, arguments);
    };
    Box2D.Collision.Shapes.b2MassData = b2MassData;

    function b2PolygonShape() {
        b2PolygonShape.b2PolygonShape.apply(this, arguments);
        if (this.constructor === b2PolygonShape) this.b2PolygonShape.apply(this, arguments);
    };
    Box2D.Collision.Shapes.b2PolygonShape = b2PolygonShape;

    function b2Shape() {
        b2Shape.b2Shape.apply(this, arguments);
        if (this.constructor === b2Shape) this.b2Shape.apply(this, arguments);
    };
    Box2D.Collision.Shapes.b2Shape = b2Shape;

    function b2Color() {
        b2Color.b2Color.apply(this, arguments);
        if (this.constructor === b2Color) this.b2Color.apply(this, arguments);
    };
    Box2D.Common.b2Color = b2Color;

    function b2Settings() {
        b2Settings.b2Settings.apply(this, arguments);
    };
    Box2D.Common.b2Settings = b2Settings;

    function b2Timer() {
        b2Timer.b2Timer.apply(this, arguments);
        if (this.constructor === b2Timer) this.b2Timer.apply(this, arguments);
    };
    Box2D.Common.b2Timer = b2Timer;

    function b2Mat22() {
        b2Mat22.b2Mat22.apply(this, arguments);
        if (this.constructor === b2Mat22) this.b2Mat22.apply(this, arguments);
    };
    Box2D.Common.Math.b2Mat22 = b2Mat22;

    function b2Mat33() {
        b2Mat33.b2Mat33.apply(this, arguments);
        if (this.constructor === b2Mat33) this.b2Mat33.apply(this, arguments);
    };
    Box2D.Common.Math.b2Mat33 = b2Mat33;

    function b2Rot() {
        b2Rot.b2Rot.apply(this, arguments);
        if (this.constructor === b2Rot) this.b2Rot.apply(this, arguments);
    };
    Box2D.Common.Math.b2Rot = b2Rot;

    function b2Math() {
        b2Math.b2Math.apply(this, arguments);
    };
    Box2D.Common.Math.b2Math = b2Math;

    function b2Sweep() {
        b2Sweep.b2Sweep.apply(this, arguments);
    };
    Box2D.Common.Math.b2Sweep = b2Sweep;

    function b2Transform() {
        b2Transform.b2Transform.apply(this, arguments);
        if (this.constructor === b2Transform) this.b2Transform.apply(this, arguments);
    };
    Box2D.Common.Math.b2Transform = b2Transform;

    function b2Vec2() {
        b2Vec2.b2Vec2.apply(this, arguments);
        if (this.constructor === b2Vec2) this.b2Vec2.apply(this, arguments);
    };
    Box2D.Common.Math.b2Vec2 = b2Vec2;

    function b2Vec3() {
        b2Vec3.b2Vec3.apply(this, arguments);
        if (this.constructor === b2Vec3) this.b2Vec3.apply(this, arguments);
    };
    Box2D.Common.Math.b2Vec3 = b2Vec3;

    function b2Body() {
        b2Body.b2Body.apply(this, arguments);
        if (this.constructor === b2Body) this.b2Body.apply(this, arguments);
    };
    Box2D.Dynamics.b2Body = b2Body;

    function b2BodyDef() {
        b2BodyDef.b2BodyDef.apply(this, arguments);
        if (this.constructor === b2BodyDef) this.b2BodyDef.apply(this, arguments);
    };
    Box2D.Dynamics.b2BodyDef = b2BodyDef;

    function b2ContactFilter() {
        b2ContactFilter.b2ContactFilter.apply(this, arguments);
    };
    Box2D.Dynamics.b2ContactFilter = b2ContactFilter;

    function b2ContactImpulse() {
        b2ContactImpulse.b2ContactImpulse.apply(this, arguments);
    };
    Box2D.Dynamics.b2ContactImpulse = b2ContactImpulse;

    function b2ContactListener() {
        b2ContactListener.b2ContactListener.apply(this, arguments);
    };
    Box2D.Dynamics.b2ContactListener = b2ContactListener;

    function b2ContactManager() {
        b2ContactManager.b2ContactManager.apply(this, arguments);
        if (this.constructor === b2ContactManager) this.b2ContactManager.apply(this, arguments);
    };
    Box2D.Dynamics.b2ContactManager = b2ContactManager;

    function b2Draw() {
        b2Draw.b2Draw.apply(this, arguments);
        if (this.constructor === b2Draw) this.b2Draw.apply(this, arguments);
    };
    Box2D.Dynamics.b2Draw = b2Draw;

    function b2DestructionListener() {
        b2DestructionListener.b2DestructionListener.apply(this, arguments);
    };
    Box2D.Dynamics.b2DestructionListener = b2DestructionListener;

    function b2Filter() {
        b2Filter.b2Filter.apply(this, arguments);
    };
    Box2D.Dynamics.b2Filter = b2Filter;

    function b2Fixture() {
        b2Fixture.b2Fixture.apply(this, arguments);
        if (this.constructor === b2Fixture) this.b2Fixture.apply(this, arguments);
    };
    Box2D.Dynamics.b2Fixture = b2Fixture;

    function b2FixtureDef() {
        b2FixtureDef.b2FixtureDef.apply(this, arguments);
        if (this.constructor === b2FixtureDef) this.b2FixtureDef.apply(this, arguments);
    };
    Box2D.Dynamics.b2FixtureDef = b2FixtureDef;

    function b2FixtureProxy() {
        b2FixtureProxy.b2FixtureProxy.apply(this, arguments);
        if (this.constructor === b2FixtureProxy) this.b2FixtureProxy.apply(this, arguments);
    };
    Box2D.Dynamics.b2FixtureProxy = b2FixtureProxy;

    function b2Island() {
        b2Island.b2Island.apply(this, arguments);
        if (this.constructor === b2Island) this.b2Island.apply(this, arguments);
    };
    Box2D.Dynamics.b2Island = b2Island;

    function b2Position() {
        b2Position.b2Position.apply(this, arguments);
    };
    Box2D.Dynamics.b2Position = b2Position;

    function b2Profile() {
        b2Profile.b2Profile.apply(this, arguments);
    };
    Box2D.Dynamics.b2Profile = b2Profile;

    function b2SolverData() {
        b2SolverData.b2SolverData.apply(this, arguments);
    };
    Box2D.Dynamics.b2SolverData = b2SolverData;

    function b2TimeStep() {
        b2TimeStep.b2TimeStep.apply(this, arguments);
    };
    Box2D.Dynamics.b2TimeStep = b2TimeStep;

    function b2Velocity() {
        b2Velocity.b2Velocity.apply(this, arguments);
    };
    Box2D.Dynamics.b2Velocity = b2Velocity;

    function b2World() {
        b2World.b2World.apply(this, arguments);
        if (this.constructor === b2World) this.b2World.apply(this, arguments);
    };
    Box2D.Dynamics.b2World = b2World;

    function b2WorldQueryWrapper() {
        b2WorldQueryWrapper.b2WorldQueryWrapper.apply(this, arguments);
    };
    Box2D.Dynamics.b2WorldQueryWrapper = b2WorldQueryWrapper;

    function b2WorldRayCastWrapper() {
        b2WorldRayCastWrapper.b2WorldRayCastWrapper.apply(this, arguments);
    };
    Box2D.Dynamics.b2WorldRayCastWrapper = b2WorldRayCastWrapper;

    function b2ChainAndCircleContact() {
        b2ChainAndCircleContact.b2ChainAndCircleContact.apply(this, arguments);
    };
    Box2D.Dynamics.Contacts.b2ChainAndCircleContact = b2ChainAndCircleContact;

    function b2ChainAndPolygonContact() {
        b2ChainAndPolygonContact.b2ChainAndPolygonContact.apply(this, arguments);
    };
    Box2D.Dynamics.Contacts.b2ChainAndPolygonContact = b2ChainAndPolygonContact;

    function b2CircleContact() {
        b2CircleContact.b2CircleContact.apply(this, arguments);
    };
    Box2D.Dynamics.Contacts.b2CircleContact = b2CircleContact;

    function b2Contact() {
        b2Contact.b2Contact.apply(this, arguments);
        if (this.constructor === b2Contact) this.b2Contact.apply(this, arguments);
    };
    Box2D.Dynamics.Contacts.b2Contact = b2Contact;

    function b2ContactEdge() {
        b2ContactEdge.b2ContactEdge.apply(this, arguments);
    };
    Box2D.Dynamics.Contacts.b2ContactEdge = b2ContactEdge;

    function b2ContactPositionConstraint() {
        b2ContactPositionConstraint.b2ContactPositionConstraint.apply(this, arguments);
        if (this.constructor === b2ContactPositionConstraint) this.b2ContactPositionConstraint.apply(this, arguments);
    };
    Box2D.Dynamics.Contacts.b2ContactPositionConstraint = b2ContactPositionConstraint;

    function b2ContactRegister() {
        b2ContactRegister.b2ContactRegister.apply(this, arguments);
    };
    Box2D.Dynamics.Contacts.b2ContactRegister = b2ContactRegister;

    function b2ContactSolver() {
        b2ContactSolver.b2ContactSolver.apply(this, arguments);
        if (this.constructor === b2ContactSolver) this.b2ContactSolver.apply(this, arguments);
    };
    Box2D.Dynamics.Contacts.b2ContactSolver = b2ContactSolver;

    function b2ContactSolverDef() {
        b2ContactSolverDef.b2ContactSolverDef.apply(this, arguments);
        if (this.constructor === b2ContactSolverDef) this.b2ContactSolverDef.apply(this, arguments);
    };
    Box2D.Dynamics.Contacts.b2ContactSolverDef = b2ContactSolverDef;

    function b2ContactVelocityConstraint() {
        b2ContactVelocityConstraint.b2ContactVelocityConstraint.apply(this, arguments);
        if (this.constructor === b2ContactVelocityConstraint) this.b2ContactVelocityConstraint.apply(this, arguments);
    };
    Box2D.Dynamics.Contacts.b2ContactVelocityConstraint = b2ContactVelocityConstraint;

    function b2EdgeAndCircleContact() {
        b2EdgeAndCircleContact.b2EdgeAndCircleContact.apply(this, arguments);
    };
    Box2D.Dynamics.Contacts.b2EdgeAndCircleContact = b2EdgeAndCircleContact;

    function b2PolygonAndCircleContact() {
        b2PolygonAndCircleContact.b2PolygonAndCircleContact.apply(this, arguments);
    };
    Box2D.Dynamics.Contacts.b2PolygonAndCircleContact = b2PolygonAndCircleContact;

    function b2EdgeAndPolygonContact() {
        b2EdgeAndPolygonContact.b2EdgeAndPolygonContact.apply(this, arguments);
    };
    Box2D.Dynamics.Contacts.b2EdgeAndPolygonContact = b2EdgeAndPolygonContact;

    function b2PolygonContact() {
        b2PolygonContact.b2PolygonContact.apply(this, arguments);
    };
    Box2D.Dynamics.Contacts.b2PolygonContact = b2PolygonContact;

    function b2PositionSolverManifold() {
        b2PositionSolverManifold.b2PositionSolverManifold.apply(this, arguments);
        if (this.constructor === b2PositionSolverManifold) this.b2PositionSolverManifold.apply(this, arguments);
    };
    Box2D.Dynamics.Contacts.b2PositionSolverManifold = b2PositionSolverManifold;

    function b2VelocityConstraintPoint() {
        b2VelocityConstraintPoint.b2VelocityConstraintPoint.apply(this, arguments);
    };
    Box2D.Dynamics.Contacts.b2VelocityConstraintPoint = b2VelocityConstraintPoint;

    function b2DistanceJoint() {
        b2DistanceJoint.b2DistanceJoint.apply(this, arguments);
        if (this.constructor === b2DistanceJoint) this.b2DistanceJoint.apply(this, arguments);
    };
    Box2D.Dynamics.Joints.b2DistanceJoint = b2DistanceJoint;

    function b2DistanceJointDef() {
        b2DistanceJointDef.b2DistanceJointDef.apply(this, arguments);
        if (this.constructor === b2DistanceJointDef) this.b2DistanceJointDef.apply(this, arguments);
    };
    Box2D.Dynamics.Joints.b2DistanceJointDef = b2DistanceJointDef;

    function b2FrictionJoint() {
        b2FrictionJoint.b2FrictionJoint.apply(this, arguments);
        if (this.constructor === b2FrictionJoint) this.b2FrictionJoint.apply(this, arguments);
    };
    Box2D.Dynamics.Joints.b2FrictionJoint = b2FrictionJoint;

    function b2FrictionJointDef() {
        b2FrictionJointDef.b2FrictionJointDef.apply(this, arguments);
        if (this.constructor === b2FrictionJointDef) this.b2FrictionJointDef.apply(this, arguments);
    };
    Box2D.Dynamics.Joints.b2FrictionJointDef = b2FrictionJointDef;

    function b2GearJoint() {
        b2GearJoint.b2GearJoint.apply(this, arguments);
        if (this.constructor === b2GearJoint) this.b2GearJoint.apply(this, arguments);
    };
    Box2D.Dynamics.Joints.b2GearJoint = b2GearJoint;

    function b2GearJointDef() {
        b2GearJointDef.b2GearJointDef.apply(this, arguments);
        if (this.constructor === b2GearJointDef) this.b2GearJointDef.apply(this, arguments);
    };
    Box2D.Dynamics.Joints.b2GearJointDef = b2GearJointDef;

    function b2Jacobian() {
        b2Jacobian.b2Jacobian.apply(this, arguments);
    };
    Box2D.Dynamics.Joints.b2Jacobian = b2Jacobian;

    function b2Joint() {
        b2Joint.b2Joint.apply(this, arguments);
        if (this.constructor === b2Joint) this.b2Joint.apply(this, arguments);
    };
    Box2D.Dynamics.Joints.b2Joint = b2Joint;

    function b2JointDef() {
        b2JointDef.b2JointDef.apply(this, arguments);
        if (this.constructor === b2JointDef) this.b2JointDef.apply(this, arguments);
    };
    Box2D.Dynamics.Joints.b2JointDef = b2JointDef;

    function b2JointEdge() {
        b2JointEdge.b2JointEdge.apply(this, arguments);
    };
    Box2D.Dynamics.Joints.b2JointEdge = b2JointEdge;

    function b2MouseJoint() {
        b2MouseJoint.b2MouseJoint.apply(this, arguments);
        if (this.constructor === b2MouseJoint) this.b2MouseJoint.apply(this, arguments);
    };
    Box2D.Dynamics.Joints.b2MouseJoint = b2MouseJoint;

    function b2MouseJointDef() {
        b2MouseJointDef.b2MouseJointDef.apply(this, arguments);
        if (this.constructor === b2MouseJointDef) this.b2MouseJointDef.apply(this, arguments);
    };
    Box2D.Dynamics.Joints.b2MouseJointDef = b2MouseJointDef;

    function b2PrismaticJoint() {
        b2PrismaticJoint.b2PrismaticJoint.apply(this, arguments);
        if (this.constructor === b2PrismaticJoint) this.b2PrismaticJoint.apply(this, arguments);
    };
    Box2D.Dynamics.Joints.b2PrismaticJoint = b2PrismaticJoint;

    function b2PrismaticJointDef() {
        b2PrismaticJointDef.b2PrismaticJointDef.apply(this, arguments);
        if (this.constructor === b2PrismaticJointDef) this.b2PrismaticJointDef.apply(this, arguments);
    };
    Box2D.Dynamics.Joints.b2PrismaticJointDef = b2PrismaticJointDef;

    function b2PulleyJoint() {
        b2PulleyJoint.b2PulleyJoint.apply(this, arguments);
        if (this.constructor === b2PulleyJoint) this.b2PulleyJoint.apply(this, arguments);
    };
    Box2D.Dynamics.Joints.b2PulleyJoint = b2PulleyJoint;

    function b2PulleyJointDef() {
        b2PulleyJointDef.b2PulleyJointDef.apply(this, arguments);
        if (this.constructor === b2PulleyJointDef) this.b2PulleyJointDef.apply(this, arguments);
    };
    Box2D.Dynamics.Joints.b2PulleyJointDef = b2PulleyJointDef;

    function b2RevoluteJoint() {
        b2RevoluteJoint.b2RevoluteJoint.apply(this, arguments);
        if (this.constructor === b2RevoluteJoint) this.b2RevoluteJoint.apply(this, arguments);
    };
    Box2D.Dynamics.Joints.b2RevoluteJoint = b2RevoluteJoint;

    function b2RevoluteJointDef() {
        b2RevoluteJointDef.b2RevoluteJointDef.apply(this, arguments);
        if (this.constructor === b2RevoluteJointDef) this.b2RevoluteJointDef.apply(this, arguments);
    };
    Box2D.Dynamics.Joints.b2RevoluteJointDef = b2RevoluteJointDef;

    function b2RopeJoint() {
        b2RopeJoint.b2RopeJoint.apply(this, arguments);
        if (this.constructor === b2RopeJoint) this.b2RopeJoint.apply(this, arguments);
    };
    Box2D.Dynamics.Joints.b2RopeJoint = b2RopeJoint;

    function b2RopeJointDef() {
        b2RopeJointDef.b2RopeJointDef.apply(this, arguments);
        if (this.constructor === b2RopeJointDef) this.b2RopeJointDef.apply(this, arguments);
    };
    Box2D.Dynamics.Joints.b2RopeJointDef = b2RopeJointDef;

    function b2WeldJoint() {
        b2WeldJoint.b2WeldJoint.apply(this, arguments);
        if (this.constructor === b2WeldJoint) this.b2WeldJoint.apply(this, arguments);
    };
    Box2D.Dynamics.Joints.b2WeldJoint = b2WeldJoint;

    function b2WeldJointDef() {
        b2WeldJointDef.b2WeldJointDef.apply(this, arguments);
        if (this.constructor === b2WeldJointDef) this.b2WeldJointDef.apply(this, arguments);
    };
    Box2D.Dynamics.Joints.b2WeldJointDef = b2WeldJointDef;

    function b2WheelJoint() {
        b2WheelJoint.b2WheelJoint.apply(this, arguments);
        if (this.constructor === b2WheelJoint) this.b2WheelJoint.apply(this, arguments);
    };
    Box2D.Dynamics.Joints.b2WheelJoint = b2WheelJoint;

    function b2WheelJointDef() {
        b2WheelJointDef.b2WheelJointDef.apply(this, arguments);
        if (this.constructor === b2WheelJointDef) this.b2WheelJointDef.apply(this, arguments);
    };
    Box2D.Dynamics.Joints.b2WheelJointDef = b2WheelJointDef;

    function b2Rope() {
        b2Rope.b2Rope.apply(this, arguments);
    };
    Box2D.Rope.b2Rope = b2Rope;

    function b2RopeDef() {
        b2RopeDef.b2RopeDef.apply(this, arguments);
    };
    Box2D.Rope.b2RopeDef = b2RopeDef;
})(); //definitions
Box2D.postDefs = [];
(function() {
    var b2CircleShape = Box2D.Collision.Shapes.b2CircleShape,
        b2EdgeShape = Box2D.Collision.Shapes.b2EdgeShape,
        b2ChainShape = Box2D.Collision.Shapes.b2ChainShape,
        b2MassData = Box2D.Collision.Shapes.b2MassData,
        b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape,
        b2Shape = Box2D.Collision.Shapes.b2Shape,
        b2Color = Box2D.Common.b2Color,
        b2Settings = Box2D.Common.b2Settings,
        b2Timer = Box2D.Common.b2Timer,
        b2Mat22 = Box2D.Common.Math.b2Mat22,
        b2Mat33 = Box2D.Common.Math.b2Mat33,
        b2Rot = Box2D.Common.Math.b2Rot,
        b2Math = Box2D.Common.Math.b2Math,
        b2Sweep = Box2D.Common.Math.b2Sweep,
        b2Transform = Box2D.Common.Math.b2Transform,
        b2Vec2 = Box2D.Common.Math.b2Vec2,
        b2Vec3 = Box2D.Common.Math.b2Vec3,
        b2AABB = Box2D.Collision.b2AABB,
        b2Collision = Box2D.Collision.b2Collision,
        b2ContactID = Box2D.Collision.b2ContactID,
        b2Distance = Box2D.Collision.b2Distance,
        b2DistanceInput = Box2D.Collision.b2DistanceInput,
        b2DistanceOutput = Box2D.Collision.b2DistanceOutput,
        b2DistanceProxy = Box2D.Collision.b2DistanceProxy,
        b2DynamicTree = Box2D.Collision.b2DynamicTree,
        b2BroadPhase = Box2D.Collision.b2BroadPhase,
        b2TreeNode = Box2D.Collision.b2TreeNode,
        b2Pair = Box2D.Collision.b2Pair,
        b2Manifold = Box2D.Collision.b2Manifold,
        b2ManifoldPoint = Box2D.Collision.b2ManifoldPoint,
        b2RayCastInput = Box2D.Collision.b2RayCastInput,
        b2RayCastOutput = Box2D.Collision.b2RayCastOutput,
        b2SeparationFunction = Box2D.Collision.b2SeparationFunction,
        b2Simplex = Box2D.Collision.b2Simplex,
        b2SimplexCache = Box2D.Collision.b2SimplexCache,
        b2SimplexVertex = Box2D.Collision.b2SimplexVertex,
        b2TOIInput = Box2D.Collision.b2TOIInput,
        b2TOIOutput = Box2D.Collision.b2TOIOutput,
        b2WorldManifold = Box2D.Collision.b2WorldManifold,
        b2ClipVertex = Box2D.Collision.b2ClipVertex,
        b2ContactFeature = Box2D.Collision.b2ContactFeature,
        IBroadPhase = Box2D.Collision.IBroadPhase,
        b2Rope = Box2D.Rope.b2Rope,
        b2RopeDef = Box2D.Rope.b2RopeDef;

    b2AABB.b2AABB = function() {
        this.lowerBound = new b2Vec2();
        this.upperBound = new b2Vec2();
    };
    b2AABB.prototype.IsValid = function() {
        var dX = this.upperBound.x - this.lowerBound.x;
        var dY = this.upperBound.y - this.lowerBound.y;
        var valid = dX >= 0.0 && dY >= 0.0;
        valid = valid && this.lowerBound.IsValid() && this.upperBound.IsValid();
        return valid;
    }
    b2AABB.prototype.GetCenter = function() {
        return new b2Vec2((this.lowerBound.x + this.upperBound.x) / 2, (this.lowerBound.y + this.upperBound.y) / 2);
    }
    b2AABB.prototype.GetExtents = function() {
        return new b2Vec2((this.upperBound.x - this.lowerBound.x) / 2, (this.upperBound.y - this.lowerBound.y) / 2);
    }
    b2AABB.prototype.GetPerimeter = function() {
        return 2.0 * ((this.upperBound.x - this.lowerBound.x) + (this.upperBound.y - this.lowerBound.y));
    }
    b2AABB.prototype.Contains = function(aabb) {
        var result = true;
        result = result && this.lowerBound.x <= aabb.lowerBound.x;
        result = result && this.lowerBound.y <= aabb.lowerBound.y;
        result = result && aabb.upperBound.x <= this.upperBound.x;
        result = result && aabb.upperBound.y <= this.upperBound.y;
        return result;
    }
    b2AABB.prototype.RayCast = function(output, input) {
        var tmin = (-Number.MAX_VALUE);
        var tmax = Number.MAX_VALUE;
        var pX = input.p1.x;
        var pY = input.p1.y;
        var dX = input.p2.x - input.p1.x;
        var dY = input.p2.y - input.p1.y;
        var absDX = Math.abs(dX);
        var absDY = Math.abs(dY);
        var normal = output.normal;
        var inv_d = 0;
        var t1 = 0;
        var t2 = 0;
        var t3 = 0;
        var s = 0; {
            if (absDX < Number.MIN_VALUE) {
                if (pX < this.lowerBound.x || this.upperBound.x < pX) return false;
            } else {
                inv_d = 1.0 / dX;
                t1 = (this.lowerBound.x - pX) * inv_d;
                t2 = (this.upperBound.x - pX) * inv_d;
                s = (-1.0);
                if (t1 > t2) {
                    t3 = t1;
                    t1 = t2;
                    t2 = t3;
                    s = 1.0;
                }
                if (t1 > tmin) {
                    normal.x = s;
                    normal.y = 0;
                    tmin = t1;
                }
                tmax = Math.min(tmax, t2);
                if (tmin > tmax) return false;
            }
        } {
            if (absDY < Number.MIN_VALUE) {
                if (pY < this.lowerBound.y || this.upperBound.y < pY) return false;
            } else {
                inv_d = 1.0 / dY;
                t1 = (this.lowerBound.y - pY) * inv_d;
                t2 = (this.upperBound.y - pY) * inv_d;
                s = (-1.0);
                if (t1 > t2) {
                    t3 = t1;
                    t1 = t2;
                    t2 = t3;
                    s = 1.0;
                }
                if (t1 > tmin) {
                    normal.y = s;
                    normal.x = 0;
                    tmin = t1;
                }
                tmax = Math.min(tmax, t2);
                if (tmin > tmax) return false;
            }
        }
        output.fraction = tmin;
        return true;
    }
    b2AABB.prototype.TestOverlap = function(other) {
        var d1X = other.lowerBound.x - this.upperBound.x;
        var d1Y = other.lowerBound.y - this.upperBound.y;
        var d2X = this.lowerBound.x - other.upperBound.x;
        var d2Y = this.lowerBound.y - other.upperBound.y;
        if (d1X > 0.0 || d1Y > 0.0) return false;
        if (d2X > 0.0 || d2Y > 0.0) return false;
        return true;
    }
    b2AABB.prototype.Combine = function(aabb1) {
        this.lowerBound.x = Math.min(aabb1.lowerBound.x, this.lowerBound.x);
        this.lowerBound.y = Math.min(aabb1.lowerBound.y, this.lowerBound.y);
        this.upperBound.x = Math.max(aabb1.upperBound.x, this.upperBound.x);
        this.upperBound.y = Math.max(aabb1.upperBound.y, this.upperBound.y);
    }
    b2AABB.prototype.Combine2 = function(aabb1, aabb2) {
        this.lowerBound.x = Math.min(aabb1.lowerBound.x, aabb2.lowerBound.x);
        this.lowerBound.y = Math.min(aabb1.lowerBound.y, aabb2.lowerBound.y);
        this.upperBound.x = Math.max(aabb1.upperBound.x, aabb2.upperBound.x);
        this.upperBound.y = Math.max(aabb1.upperBound.y, aabb2.upperBound.y);
    }
    b2Collision.b2Collision = function() {};
    b2Collision.ClipSegmentToLine = function(vOut, vIn, normal, offset, vertexIndexA) {
        var cv;
        var numOut = 0;
        cv = vIn[0];
        var vIn0 = cv.v;
        cv = vIn[1];
        var vIn1 = cv.v;
        var distance0 = normal.x * vIn0.x + normal.y * vIn0.y - offset;
        var distance1 = normal.x * vIn1.x + normal.y * vIn1.y - offset;
        if (distance0 <= 0.0) vOut[numOut++].Set(vIn[0]);
        if (distance1 <= 0.0) vOut[numOut++].Set(vIn[1]);
        if (distance0 * distance1 < 0.0) {
            var interp = distance0 / (distance0 - distance1);
            cv = vOut[numOut];
            var tVec = cv.v;
            tVec.x = vIn0.x + interp * (vIn1.x - vIn0.x);
            tVec.y = vIn0.y + interp * (vIn1.y - vIn0.y);
            cv = vOut[numOut];
            cv.id.cf.indexA = vertexIndexA;
            cv.id.cf.indexB = vIn[0].id.cf.indexB;
            cv.id.cf.typeA = b2ContactFeature.e_vertex;
            cv.id.cf.typeB = b2ContactFeature.e_face;
            ++numOut;
        }

        return numOut;
    }
    b2Collision.EdgeSeparation = function(poly1, xf1, edge1, poly2, xf2) {
        var vertices1 = poly1.m_vertices;
        var normals1 = poly1.m_normals;
        var count2 = parseInt(poly2.m_vertexCount);
        var vertices2 = poly2.m_vertices;
        var tRot;
        var tVec;
        tRot = xf1.q;
        tVec = normals1[edge1];
        var normal1WorldX = (tRot.c * tVec.x - tRot.s * tVec.y);
        var normal1WorldY = (tRot.s * tVec.x + tRot.c * tVec.y);
        tRot = xf2.q;
        var normal1X = tRot.c * normal1WorldX + tRot.s * normal1WorldY;
        var normal1Y = -tRot.s * normal1WorldX + tRot.c * normal1WorldY;

        var index = 0;
        var minDot = Number.MAX_VALUE;
        var dot;
        for (var i = 0; i < count2; ++i) {
            tVec = vertices2[i];
            dot = tVec.x * normal1X + tVec.y * normal1Y;
            if (dot < minDot) {
                minDot = dot;
                index = i;
            }
        }
        tVec = vertices1[edge1];
        tRot = xf1.q;
        var v1X = tRot.c * tVec.x - tRot.s * tVec.y + xf1.p.x;
        var v1Y = tRot.s * tVec.x + tRot.c * tVec.y + xf1.p.y;
        tVec = vertices2[index];
        tRot = xf2.q;
        var v2X = tRot.c * tVec.x - tRot.s * tVec.y + xf2.p.x;
        var v2Y = tRot.s * tVec.x + tRot.c * tVec.y + xf2.p.y;
        v2X -= v1X;
        v2Y -= v1Y;
        var separation = v2X * normal1WorldX + v2Y * normal1WorldY;
        return separation;
    }
    b2Collision.FindMaxSeparation = function(edgeIndex0, poly1, xf1, poly2, xf2) {
        var count1 = parseInt(poly1.m_vertexCount);
        var normals1 = poly1.m_normals;

        var tVec, tRot;
        tVec = poly2.m_centroid;
        tRot = xf2.q;
        var dX = xf2.p.x + tRot.c * tVec.x - tRot.s * tVec.y;
        var dY = xf2.p.y + tRot.s * tVec.x + tRot.c * tVec.y;
        tVec = poly1.m_centroid;
        tRot = xf1.q;
        dX -= xf1.p.x + tRot.c * tVec.x - tRot.s * tVec.y;
        dY -= xf1.p.y + tRot.s * tVec.x + tRot.c * tVec.y;
        var dLocal1X = tRot.c * dX + tRot.s * dY;
        var dLocal1Y = -tRot.s * dX + tRot.c * dY;

        var edge = 0;
        var maxDot = (-Number.MAX_VALUE);
        for (var i = 0; i < count1; ++i) {
            tVec = normals1[i];
            var dot = tVec.x * dLocal1X + tVec.y * dLocal1Y;
            if (dot > maxDot) {
                maxDot = dot;
                edge = i;
            }
        }
        var s = b2Collision.EdgeSeparation(poly1, xf1, edge, poly2, xf2);
        var prevEdge = parseInt(edge - 1 >= 0 ? edge - 1 : count1 - 1);
        var sPrev = b2Collision.EdgeSeparation(poly1, xf1, prevEdge, poly2, xf2);
        var nextEdge = parseInt(edge + 1 < count1 ? edge + 1 : 0);
        var sNext = b2Collision.EdgeSeparation(poly1, xf1, nextEdge, poly2, xf2);

        var bestEdge = 0;
        var bestSeparation = 0;
        var increment = 0;
        if (sPrev > s && sPrev > sNext) {
            increment = (-1);
            bestEdge = prevEdge;
            bestSeparation = sPrev;
        } else if (sNext > s) {
            increment = 1;
            bestEdge = nextEdge;
            bestSeparation = sNext;
        } else {
            edgeIndex0[0] = edge;
            return s;
        }
        while (true) {
            if (increment == (-1)) edge = bestEdge - 1 >= 0 ? bestEdge - 1 : count1 - 1;
            else edge = bestEdge + 1 < count1 ? bestEdge + 1 : 0;
            s = b2Collision.EdgeSeparation(poly1, xf1, edge, poly2, xf2);
            if (s > bestSeparation) {
                bestEdge = edge;
                bestSeparation = s;
            } else {
                break;
            }
        }
        edgeIndex0[0] = bestEdge;
        return bestSeparation;
    }
    b2Collision.FindIncidentEdge = function(c, poly1, xf1, edge1, poly2, xf2) {
        var normals1 = poly1.m_normals;
        var count2 = parseInt(poly2.m_vertexCount);
        var vertices2 = poly2.m_vertices;
        var normals2 = poly2.m_normals;

        //b2Settings.b2Assert(0 <= edge1 && edge1 < poly1.m_vertexCount);

        var tQ;
        var tVec;
        tQ = xf1.q;
        tVec = normals1[edge1];
        var normal1WorldX = tQ.c * tVec.x - tQ.s * tVec.y;
        var normal1WorldY = tQ.s * tVec.x + tQ.c * tVec.y;
        tQ = xf2.q;
        var normal1X = tQ.c * normal1WorldX + tQ.s * normal1WorldY;
        var normal1Y = -tQ.s * normal1WorldX + tQ.c * normal1WorldY;

        var index = 0;
        var minDot = Number.MAX_VALUE;
        for (var i = 0; i < count2; ++i) {
            tVec = normals2[i];
            var dot = normal1X * tVec.x + normal1Y * tVec.y;
            if (dot < minDot) {
                minDot = dot;
                index = i;
            }
        }
        var i1 = parseInt(index);
        var i2 = parseInt(i1 + 1 < count2 ? i1 + 1 : 0);

        var tClip = c[0];
        tQ = xf2.q;
        tVec = vertices2[i1];
        tClip.v.x = (tQ.c * tVec.x - tQ.s * tVec.y) + xf2.p.x;
        tClip.v.y = (tQ.s * tVec.x + tQ.c * tVec.y) + xf2.p.y;
        tClip.id.cf.indexA = edge1;
        tClip.id.cf.indexB = i1;
        tClip.id.cf.typeA = b2ContactFeature.e_face;
        tClip.id.cf.typeB = b2ContactFeature.e_vertex;
        tClip = c[1];
        tQ = xf2.q;
        tVec = vertices2[i2];
        tClip.v.x = (tQ.c * tVec.x - tQ.s * tVec.y) + xf2.p.x;
        tClip.v.y = (tQ.s * tVec.x + tQ.c * tVec.y) + xf2.p.y;
        tClip.id.cf.indexA = edge1;
        tClip.id.cf.indexB = i2;
        tClip.id.cf.typeA = b2ContactFeature.e_face;
        tClip.id.cf.typeB = b2ContactFeature.e_vertex;
    }
    b2Collision.MakeClipPointVector = function() {
        var r = new Vector(2);
        r[0] = new b2ClipVertex();
        r[1] = new b2ClipVertex();
        return r;
    }
    b2Collision.CollidePolygons = function(manifold, polyA, xfA, polyB, xfB) {
        var cv;
        manifold.pointCount = 0;
        var totalRadius = polyA.m_radius + polyB.m_radius;

        var edgeA = 0;
        b2Collision.s_edgeAO[0] = edgeA;
        var separationA = b2Collision.FindMaxSeparation(b2Collision.s_edgeAO, polyA, xfA, polyB, xfB);
        edgeA = b2Collision.s_edgeAO[0];
        if (separationA > totalRadius) return;

        var edgeB = 0;
        b2Collision.s_edgeBO[0] = edgeB;
        var separationB = b2Collision.FindMaxSeparation(b2Collision.s_edgeBO, polyB, xfB, polyA, xfA);
        edgeB = b2Collision.s_edgeBO[0];
        if (separationB > totalRadius) return;

        var poly1;
        var poly2;
        var xf1;
        var xf2;
        var edge1 = 0;
        var flip = 0;
        var k_relativeTol = 0.98;
        var k_absoluteTol = 0.001;

        if (separationB > k_relativeTol * separationA + k_absoluteTol) {
            poly1 = polyB;
            poly2 = polyA;
            xf1 = xfB;
            xf2 = xfA;
            edge1 = edgeB;
            manifold.type = b2Manifold.e_faceB;
            flip = 1;
        } else {
            poly1 = polyA;
            poly2 = polyB;
            xf1 = xfA;
            xf2 = xfB;
            edge1 = edgeA;
            manifold.type = b2Manifold.e_faceA;
            flip = 0;
        }
        var incidentEdge = b2Collision.s_incidentEdge;
        b2Collision.FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);
        var count1 = parseInt(poly1.m_vertexCount);
        var vertices1 = poly1.m_vertices;
        var iv1 = edge1;
        var iv2 = (edge1 + 1 < count1) ? parseInt(edge1 + 1) : 0;

        var local_v11 = vertices1[iv1];
        var local_v12 = vertices1[iv2];
        var localTangent = b2Collision.s_localTangent;
        localTangent.Set(local_v12.x - local_v11.x, local_v12.y - local_v11.y);
        localTangent.Normalize();
        var localNormal = b2Collision.s_localNormal;
        localNormal.x = localTangent.y;
        localNormal.y = (-localTangent.x);
        var planePoint = b2Collision.s_planePoint;
        planePoint.Set(0.5 * (local_v11.x + local_v12.x), 0.5 * (local_v11.y + local_v12.y));
        var tangent = b2Collision.s_tangent;
        var tRot = xf1.q;
        tangent.x = (tRot.c * localTangent.x - tRot.s * localTangent.y);
        tangent.y = (tRot.s * localTangent.x + tRot.c * localTangent.y);
        var tangent2 = b2Collision.s_tangent2;
        tangent2.x = (-tangent.x);
        tangent2.y = (-tangent.y);
        var normal = b2Collision.s_normal;
        normal.x = tangent.y;
        normal.y = (-tangent.x);
        var v11 = b2Collision.s_v11;
        var v12 = b2Collision.s_v12;
        v11.x = xf1.p.x + (tRot.c * local_v11.x - tRot.s * local_v11.y);
        v11.y = xf1.p.y + (tRot.s * local_v11.x + tRot.c * local_v11.y);
        v12.x = xf1.p.x + (tRot.c * local_v12.x - tRot.s * local_v12.y);
        v12.y = xf1.p.y + (tRot.s * local_v12.x + tRot.c * local_v12.y);
        var frontOffset = normal.x * v11.x + normal.y * v11.y;
        var sideOffset1 = (-tangent.x * v11.x) - tangent.y * v11.y + totalRadius;
        var sideOffset2 = tangent.x * v12.x + tangent.y * v12.y + totalRadius;
        var clipPoints1 = b2Collision.s_clipPoints1;
        var clipPoints2 = b2Collision.s_clipPoints2;
        var np = 0;
        np = b2Collision.ClipSegmentToLine(clipPoints1, incidentEdge, tangent2, sideOffset1, iv1);
        if (np < 2) return;
        np = b2Collision.ClipSegmentToLine(clipPoints2, clipPoints1, tangent, sideOffset2, iv2);
        if (np < 2) return;
        manifold.localNormal.SetV(localNormal);
        manifold.localPoint.SetV(planePoint);
        var pointCount = 0;
        for (var i = 0; i < b2Settings.b2_maxManifoldPoints; ++i) {
            cv = clipPoints2[i];
            var separation = normal.x * cv.v.x + normal.y * cv.v.y - frontOffset;
            if (separation <= totalRadius) {
                var cp = manifold.points[pointCount];
                cp.localPoint.x = xf2.q.c * (cv.v.x - xf2.p.x) + xf2.q.s * (cv.v.y - xf2.p.y);
                cp.localPoint.y = -xf2.q.s * (cv.v.x - xf2.p.x) + xf2.q.c * (cv.v.y - xf2.p.y);
                cp.id.Set(cv.id);
//                cp.id.features.flip = flip;
                if (flip) {
                    var id = cp.id;
                    cp.id.cf.indexA = id.cf.indexB;
                    cp.id.cf.indexB = id.cf.indexA;
                    cp.id.cf.typeA = id.cf.typeB;
                    cp.id.cf.typeB = id.cf.typeA;
                }
                ++pointCount;
            }
        }
        manifold.pointCount = pointCount;
    }
    b2Collision.CollideCircles = function(manifold, circleA, xfA, circleB, xfB) {
        manifold.pointCount = 0;
        var tRot;
        var tVec;
        tRot = xfA.q;
        tVec = circleA.m_p;
        var p1X = xfA.p.x + (tRot.c * tVec.x - tRot.s * tVec.y);
        var p1Y = xfA.p.y + (tRot.s * tVec.x + tRot.c * tVec.y);
        tRot = xfB.q;
        tVec = circleB.m_p;
        var p2X = xfB.p.x + (tRot.c * tVec.x - tRot.s * tVec.y);
        var p2Y = xfB.p.y + (tRot.s * tVec.x + tRot.c * tVec.y);
        var dX = p2X - p1X;
        var dY = p2Y - p1Y;
        var distSqr = dX * dX + dY * dY;
        var radius = circleA.m_radius + circleB.m_radius;
        if (distSqr > radius * radius) {
            return;
        }
        manifold.type = b2Manifold.e_circles;
        manifold.localPoint.SetV(circleA.m_p);
        manifold.localNormal.SetZero();
        manifold.pointCount = 1;

        manifold.points[0].localPoint.SetV(circleB.m_p);
        manifold.points[0].id.key = 0;
    }
    b2Collision.CollidePolygonAndCircle = function(manifold, polygonA, xfA, circleB, xfB) {
        manifold.pointCount = 0;

        // Compute circle position in the frame of the polygon.
        var c = b2Math.MulXV(xfB, circleB.m_p);
        var cLocal = b2Math.MulTXV(xfA, c);

        // Find the min separating edge.
        var normalIndex = 0;
        var separation = -b2Settings.b2_maxFloat;
        var radius = polygonA.m_radius + circleB.m_radius;
        var vertexCount = polygonA.m_vertexCount;
        var vertices = polygonA.m_vertices;
        var normals = polygonA.m_normals;

        for (var i = 0; i < vertexCount; ++i) {
            var s = b2Math.DotVV(normals[i], b2Math.SubtractVV(cLocal, vertices[i]));

            if (s > radius) {
                // Early out.
                return;
            }

            if (s > separation) {
                separation = s;
                normalIndex = i;
            }
        }

        // Vertices that subtend the incident face.
        var vertIndex1 = normalIndex;
        var vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
        var v1 = vertices[vertIndex1];
        var v2 = vertices[vertIndex2];

        // If the center is inside the polygon ...
        if (separation < b2Settings.b2_epsilon) {
            manifold.pointCount = 1;
            manifold.type = b2Manifold.e_faceA;
            manifold.localNormal.SetV(normals[normalIndex]);
            manifold.localPoint = b2Math.MulFV(0.5, b2Math.AddVV(v1, v2));
            manifold.points[0].localPoint.SetV(circleB.m_p);
            manifold.points[0].id.key = 0;
            return;
        }

        // Compute barycentric coordinates
        var u1 = b2Math.DotVV(b2Math.SubtractVV(cLocal, v1), b2Math.SubtractVV(v2, v1));
        var u2 = b2Math.DotVV(b2Math.SubtractVV(cLocal, v2), b2Math.SubtractVV(v1, v2));
        if (u1 <= 0.0) {
            if (b2Math.DistanceSquared(cLocal, v1) > radius * radius) {
                return;
            }

            manifold.pointCount = 1;
            manifold.type = b2Manifold.e_faceA;
            manifold.localNormal = b2Math.SubtractVV(cLocal, v1);
            manifold.localNormal.Normalize();
            manifold.localPoint.SetV(v1);
            manifold.points[0].localPoint.SetV(circleB.m_p);
            manifold.points[0].id.key = 0;
        } else if (u2 <= 0.0) {
            if (b2Math.DistanceSquared(cLocal, v2) > radius * radius) {
                return;
            }

            manifold.pointCount = 1;
            manifold.type = b2Manifold.e_faceA;
            manifold.localNormal = b2Math.SubtractVV(cLocal, v2);
            manifold.localNormal.Normalize();
            manifold.localPoint.SetV(v2);
            manifold.points[0].localPoint.SetV(circleB.m_p);
            manifold.points[0].id.key = 0;
        } else {
            var faceCenter = b2Math.MulFV(0.5, b2Math.AddVV(v1, v2));
            var separation = b2Math.DotVV(b2Math.SubtractVV(cLocal, faceCenter), normals[vertIndex1]);
            if (separation > radius) {
                return;
            }

            manifold.pointCount = 1;
            manifold.type = b2Manifold.e_faceA;
            manifold.localNormal.SetV(normals[vertIndex1]);
            manifold.localPoint.SetV(faceCenter);
            manifold.points[0].localPoint.SetV(circleB.m_p);
            manifold.points[0].id.key = 0;
        }
    }
    b2Collision.CollideEdgeAndCircle = function(manifold, edgeA, xfA, circleB, xfB) {
        manifold.pointCount = 0;

        // Compute circle in frame of edge
        var Q = b2Math.MulTXV(xfA, b2Math.MulXV(xfB, circleB.m_p));

        var A = edgeA.m_vertex1, B = edgeA.m_vertex2;
        var e = b2Math.SubtractVV(B, A);

        // Barycentric coordinates
        var u = b2Math.DotVV(e, b2Math.SubtractVV(B, Q));
        var v = b2Math.DotVV(e, b2Math.SubtractVV(Q, A));

        var radius = edgeA.m_radius + circleB.m_radius;

        var cid = new b2ContactID();
        cid.cf.indexB = 0;
        cid.cf.typeB = b2ContactFeature.e_vertex;

        // Region A
        if (v <= 0.0) {
            var P = A;
            var d = b2Math.SubtractVV(Q, P);
            var dd = b2Math.DotVV(d, d);
            if (dd > radius * radius) {
                return;
            }

            // Is there an edge connected to A?
            if (edgeA.m_hasVertex0) {
                var A1 = edgeA.m_vertex0;
                var B1 = A;
                var e1 = b2Math.SubtractVV(B1, A1);
                var u1 = b2Math.DotVV(e1, b2Math.SubtractVV(B1, Q));

                // Is the circle in Region AB of the previous edge?
                if (u1 > 0.0) {
                    return;
                }
            }

            cid.cf.indexA = 0;
            cid.cf.typeA = b2ContactFeature.e_vertex;
            manifold.pointCount = 1;
            manifold.type = b2Manifold.e_circles;
            manifold.localNormal.SetZero();
            manifold.localPoint = P;
            manifold.points[0].id.key = cid.key;
            manifold.points[0].localPoint.SetV(circleB.m_p);
            return;
        }

        // Region B
        if (u <= 0.0) {
            var P = B;
            var d = b2Math.SubtractVV(Q, P);
            var dd = b2Math.DotVV(d, d);
            if (dd > radius * radius) {
                return;
            }

            // Is there an edge connected to B?
            if (edgeA.m_hasVertex3) {
                var B2 = edgeA.m_vertex3;
                var A2 = B;
                var e2 = b2Math.SubtractVV(B2, A2);
                var v2 = b2Math.DotVV(e2, b2Math.SubtractVV(Q, A2));

                // Is the circle in Region AB of the next edge?
                if (v2 > 0.0) {
                    return;
                }
            }

            cid.cf.indexA = 1;
            cid.cf.typeA = b2ContactFeature.e_vertex;
            manifold.pointCount = 1;
            manifold.type = b2Manifold.e_circles;
            manifold.localNormal.SetZero();
            manifold.localPoint = P;
            manifold.points[0].id.key = cid.key;
            manifold.points[0].localPoint.SetV(circleB.m_p);
            return;
        }

        // Region AB
        var den = b2Math.DotVV(e, e);
        b2Settings.b2Assert(den > 0.0);
        var P = b2Math.MulFV(1.0 / den, b2Math.AddVV(b2Math.MulFV(u, A), b2Math.MulFV(v, B)));
        var d = b2Math.SubtractVV(Q, P);
        var dd = b2Math.DotVV(d, d);
        if (dd > radius * radius) {
            return;
        }

        var n = new b2Vec2(-e.y, e.x);
        if (b2Math.DotVV(n, b2Math.SubtractVV(Q, A)) < 0.0) {
            n.Set(-n.x, -n.y);
        }
        n.Normalize();

        cid.cf.indexA = 0;
        cid.cf.typeA = b2ContactFeature.e_face;
        manifold.pointCount = 1;
        manifold.type = b2Manifold.e_faceA;
        manifold.localNormal = n;
        manifold.localPoint = A;
        manifold.points[0].id.key = cid.key;
        manifold.points[0].localPoint.SetV(circleB.m_p);
    }
    function b2EPAxis() {
        b2EPAxis.b2EPAxis.apply(this, arguments);
    };
    b2EPAxis.b2EPAxis = function() {}
    b2EPAxis.prototype.Copy = function() {
        var copy = new b2EPAxis();
        copy.type = this.type;
        copy.index = this.index;
        copy.separation = this.separation;
        return copy;
    }
    Box2D.postDefs.push(function() {
        b2EPAxis.e_unknown = 0;
        b2EPAxis.e_edgeA = 1;
        b2EPAxis.e_edgeB = 2;
    });
    function b2TempPolygon() {
        b2TempPolygon.b2TempPolygon.apply(this, arguments);
    };
    b2TempPolygon.b2TempPolygon = function() {
        this.vertices = new Vector();
        this.normals = new Vector();
        for (var i = 0; i < b2Settings.b2_maxPolygonVertices; ++i)
        {
            this.vertices[i] = new b2Vec2();
            this.normals[i] = new b2Vec2();
        }
        this.count = 0;
    }
    function b2ReferenceFace() {
        b2ReferenceFace.b2ReferenceFace.apply(this, arguments);
    };
    b2ReferenceFace.b2ReferenceFace = function() {
        this.v1 = new b2Vec2();
        this.v2 = new b2Vec2();
        this.normal = new b2Vec2();
        this.sideNormal1 = new b2Vec2();
        this.sideNormal2 = new b2Vec2();
    }
    function b2EPCollider() {
        b2EPCollider.b2EPCollider.apply(this, arguments);
    };
    b2EPCollider.b2EPCollider = function() {
        this.m_polygonB = new b2TempPolygon;
        this.m_xf = new b2Transform;
        this.m_centroidB = new b2Vec2();
        this.m_v0 = new b2Vec2();
        this.m_v1 = new b2Vec2();
        this.m_v2 = new b2Vec2();
        this.m_v3 = new b2Vec2();
        this.m_normal0 = new b2Vec2();
        this.m_normal1 = new b2Vec2();
        this.m_normal2 = new b2Vec2();
        this.m_normal = new b2Vec2();
        this.m_lowerLimit = new b2Vec2();
        this.m_upperLimit = new b2Vec2();
    };
    b2EPCollider.prototype.Collide = function(manifold, edgeA, xfA, polygonB, xfB) {
        this.m_xf = b2Math.MulTXX(xfA, xfB);

        this.m_centroidB = b2Math.MulXV(this.m_xf, polygonB.m_centroid);

        this.m_v0.SetV(edgeA.m_vertex0);
        this.m_v1.SetV(edgeA.m_vertex1);
        this.m_v2.SetV(edgeA.m_vertex2);
        this.m_v3.SetV(edgeA.m_vertex3);
        var hasVertex0 = edgeA.m_hasVertex0;
        var hasVertex3 = edgeA.m_hasVertex3;
    
        var edge1 = b2Math.SubtractVV(this.m_v2, this.m_v1);
        edge1.Normalize();
        this.m_normal1.Set(edge1.y, -edge1.x);
        var offset1 = b2Math.DotVV(this.m_normal1, b2Math.SubtractVV(this.m_centroidB, this.m_v1));
        var offset0 = 0.0, offset2 = 0.0;
        var convex1 = false, convex2 = false;
    
        // Is there a preceding edge?
        if (hasVertex0) {
            var edge0 = b2Math.SubtractVV(this.m_v1, this.m_v0);
            edge0.Normalize();
            this.m_normal0.Set(edge0.y, -edge0.x);
            convex1 = b2Math.CrossVV(edge0, edge1) >= 0.0;
            offset0 = b2Math.DotVV(this.m_normal0, b2Math.SubtractVV(this.m_centroidB, this.m_v0));
        }
    
        // Is there a following edge?
        if (hasVertex3) {
            var edge2 = b2Math.SubtractVV(this.m_v3, this.m_v2);
            edge2.Normalize();
            this.m_normal2.Set(edge2.y, -edge2.x);
            convex2 = b2Math.CrossVV(edge1, edge2) > 0.0;
            offset2 = b2Math.DotVV(this.m_normal2, b2Math.SubtractVV(this.m_centroidB, this.m_v2));
        }
    
        // Determine front or back collision. Determine collision normal limits.
        if (hasVertex0 && hasVertex3) {
            if (convex1 && convex2) {
                this.m_front = offset0 >= 0.0 || offset1 >= 0.0 || offset2 >= 0.0;
                if (this.m_front) {
                    this.m_normal.SetV(this.m_normal1);
                    this.m_lowerLimit.SetV(this.m_normal0);
                    this.m_upperLimit.SetV(this.m_normal2);
                } else {
                    this.m_normal = this.m_normal1.GetNegative();
                    this.m_lowerLimit = this.m_normal1.GetNegative();
                    this.m_upperLimit = this.m_normal1.GetNegative();
                }
            } else if (convex1) {
                this.m_front = offset0 >= 0.0 || (offset1 >= 0.0 && offset2 >= 0.0);
                if (this.m_front) {
                    this.m_normal.SetV(this.m_normal1);
                    this.m_lowerLimit.SetV(this.m_normal0);
                    this.m_upperLimit.SetV(this.m_normal1);
                } else {
                    this.m_normal = this.m_normal1.GetNegative();
                    this.m_lowerLimit = this.m_normal2.GetNegative();
                    this.m_upperLimit = this.m_normal1.GetNegative();
                }
            } else if (convex2) {
                this.m_front = offset2 >= 0.0 || (offset0 >= 0.0 && offset1 >= 0.0);
                if (this.m_front) {
                    this.m_normal.SetV(this.m_normal1);
                    this.m_lowerLimit.SetV(this.m_normal1);
                    this.m_upperLimit.SetV(this.m_normal2);
                } else {
                    this.m_normal = this.m_normal1.GetNegative();
                    this.m_lowerLimit = this.m_normal1.GetNegative();
                    this.m_upperLimit = this.m_normal0.GetNegative();
                }
            } else {
                this.m_front = offset0 >= 0.0 && offset1 >= 0.0 && offset2 >= 0.0;
                if (this.m_front) {
                    this.m_normal.SetV(this.m_normal1);
                    this.m_lowerLimit.SetV(this.m_normal1);
                    this.m_upperLimit.SetV(this.m_normal1);
                } else {
                    this.m_normal = this.m_normal1.GetNegative();
                    this.m_lowerLimit = this.m_normal2.GetNegative();
                    this.m_upperLimit = this.m_normal0.GetNegative();
                }
            }
        } else if (hasVertex0) {
            if (convex1) {
                this.m_front = offset0 >= 0.0 || offset1 >= 0.0;
                if (this.m_front) {
                    this.m_normal.SetV(this.m_normal1);
                    this.m_lowerLimit.SetV(this.m_normal0);
                    this.m_upperLimit = this.m_normal1.GetNegative();
                } else {
                    this.m_normal = this.m_normal1.GetNegative();
                    this.m_lowerLimit = this.m_normal1.Copy();
                    this.m_upperLimit = this.m_normal1.GetNegative();
                }
            } else {
                this.m_front = offset0 >= 0.0 && offset1 >= 0.0;
                if (this.m_front) {
                    this.m_normal.SetV(this.m_normal1);
                    this.m_lowerLimit.SetV(this.m_normal1);
                    this.m_upperLimit = this.m_normal1.GetNegative();
                } else {
                    this.m_normal = this.m_normal1.GetNegative();
                    this.m_lowerLimit.SetV(this.m_normal1);
                    this.m_upperLimit = this.m_normal0.GetNegative();
                }
            }
        } else if (hasVertex3) {
            if (convex2) {
                this.m_front = offset1 >= 0.0 || offset2 >= 0.0;
                if (this.m_front) {
                    this.m_normal.SetV(this.m_normal1);
                    this.m_lowerLimit = this.m_normal1.GetNegative();
                    this.m_upperLimit.SetV(this.m_normal2);
                } else {
                    this.m_normal = this.m_normal1.GetNegative();
                    this.m_lowerLimit = this.m_normal1.GetNegative();
                    this.m_upperLimit.SetV(this.m_normal1);
                }
            } else {
                this.m_front = offset1 >= 0.0 && offset2 >= 0.0;
                if (this.m_front) {
                    this.m_normal.SetV(this.m_normal1);
                    this.m_lowerLimit = this.m_normal1.GetNegative();
                    this.m_upperLimit.SetV(this.m_normal1);
                } else {
                    this.m_normal = this.m_normal1.GetNegative();
                    this.m_lowerLimit = this.m_normal2.GetNegative();
                    this.m_upperLimit.SetV(this.m_normal1);
                }
            }       
        } else {
            this.m_front = offset1 >= 0.0;
            if (this.m_front) {
                this.m_normal.SetV(this.m_normal1);
                this.m_lowerLimit = this.m_normal1.GetNegative();
                this.m_upperLimit = this.m_normal1.GetNegative();
            } else {
                this.m_normal = this.m_normal1.GetNegative();
                this.m_lowerLimit.SetV(this.m_normal1);
                this.m_upperLimit.SetV(this.m_normal1);
            }
        }
    
        // Get polygonB in frameA
        this.m_polygonB.count = polygonB.m_vertexCount;
        for (var i = 0; i < polygonB.m_vertexCount; ++i) {
            this.m_polygonB.vertices[i] = b2Math.MulXV(this.m_xf, polygonB.m_vertices[i]);
            this.m_polygonB.normals[i] = b2Math.MulRV(this.m_xf.q, polygonB.m_normals[i]);
        }
    
        this.m_radius = 2.0 * b2Settings.b2_polygonRadius;
    
        manifold.pointCount = 0;
    
        var edgeAxis = this.ComputeEdgeSeparation();
    
        // If no valid normal can be found than this edge should not collide.
        if (edgeAxis.type == b2EPAxis.e_unknown) {
            return;
        }
    
        if (edgeAxis.separation > this.m_radius) {
            return;
        }
    
        var polygonAxis = this.ComputePolygonSeparation();
        if (polygonAxis.type != b2EPAxis.e_unknown && polygonAxis.separation > this.m_radius) {
            return;
        }
    
        // Use hysteresis for jitter reduction.
        var k_relativeTol = 0.98;
        var k_absoluteTol = 0.001;
    
        var primaryAxis = new b2EPAxis();
        if (polygonAxis.type == b2EPAxis.e_unknown) {
            primaryAxis = edgeAxis.Copy();
        } else if (polygonAxis.separation > k_relativeTol * edgeAxis.separation + k_absoluteTol) {
            primaryAxis = polygonAxis.Copy();
        } else {
            primaryAxis = edgeAxis.Copy();
        }
    
        var ie = new Vector();
        ie[0] = new b2ClipVertex();
        ie[1] = new b2ClipVertex();
        var rf = new b2ReferenceFace();
        if (primaryAxis.type == b2EPAxis.e_edgeA) {
            manifold.type = b2Manifold.e_faceA;
        
            // Search for the polygon normal that is most anti-parallel to the edge normal.
            var bestIndex = 0;
            var bestValue = b2Math.DotVV(this.m_normal, this.m_polygonB.normals[0]);
            for (var i = 1; i < this.m_polygonB.count; ++i) {
                var value = b2Math.DotVV(this.m_normal, this.m_polygonB.normals[i]);
                if (value < bestValue) {
                    bestValue = value;
                    bestIndex = i;
                }
            }
        
            var i1 = bestIndex;
            var i2 = i1 + 1 < this.m_polygonB.count ? i1 + 1 : 0;
        
            ie[0].v = this.m_polygonB.vertices[i1].Copy();
            ie[0].id.cf.indexA = 0;
            ie[0].id.cf.indexB = i1;
            ie[0].id.cf.typeA = b2ContactFeature.e_face;
            ie[0].id.cf.typeB = b2ContactFeature.e_vertex;
        
            ie[1].v = this.m_polygonB.vertices[i2].Copy();
            ie[1].id.cf.indexA = 0;
            ie[1].id.cf.indexB = i2;
            ie[1].id.cf.typeA = b2ContactFeature.e_face;
            ie[1].id.cf.typeB = b2ContactFeature.e_vertex;
        
            if (this.m_front) {
                rf.i1 = 0;
                rf.i2 = 1;
                rf.v1 = this.m_v1.Copy();
                rf.v2 = this.m_v2.Copy();
                rf.normal = this.m_normal1.Copy();
            } else {
                rf.i1 = 1;
                rf.i2 = 0;
                rf.v1 = this.m_v2.Copy();
                rf.v2 = this.m_v1.Copy();
                rf.normal = this.m_normal1.GetNegative();
            }       
        } else {
            manifold.type = b2Manifold.e_faceB;
        
            ie[0].v = this.m_v1.Copy();
            ie[0].id.cf.indexA = 0;
            ie[0].id.cf.indexB = primaryAxis.index;
            ie[0].id.cf.typeA = b2ContactFeature.e_vertex;
            ie[0].id.cf.typeB = b2ContactFeature.e_face;
        
            ie[1].v = this.m_v2.Copy();
            ie[1].id.cf.indexA = 0;
            ie[1].id.cf.indexB = primaryAxis.index;     
            ie[1].id.cf.typeA = b2ContactFeature.e_vertex;
            ie[1].id.cf.typeB = b2ContactFeature.e_face;
        
            rf.i1 = primaryAxis.index;
            rf.i2 = rf.i1 + 1 < this.m_polygonB.count ? rf.i1 + 1 : 0;
            rf.v1 = this.m_polygonB.vertices[rf.i1].Copy();
            rf.v2 = this.m_polygonB.vertices[rf.i2].Copy();
            rf.normal = this.m_polygonB.normals[rf.i1].Copy();
        }
    
        rf.sideNormal1.Set(rf.normal.y, -rf.normal.x);
        rf.sideNormal2 = rf.sideNormal1.GetNegative();
        rf.sideOffset1 = b2Math.DotVV(rf.sideNormal1, rf.v1);
        rf.sideOffset2 = b2Math.DotVV(rf.sideNormal2, rf.v2);
    
        // Clip incident edge against extruded edge1 side edges.
        var clipPoints1 = new Vector();
        clipPoints1[0] = new b2ClipVertex();
        clipPoints1[1] = new b2ClipVertex();
        var clipPoints2 = new Vector();
        clipPoints2[0] = new b2ClipVertex();
        clipPoints2[1] = new b2ClipVertex();
        var np;
    
        // Clip to box side 1
        np = b2Collision.ClipSegmentToLine(clipPoints1, ie, rf.sideNormal1, rf.sideOffset1, rf.i1);
    
        if (np < b2Settings.b2_maxManifoldPoints) {
            return;
        }
    
        // Clip to negative box side 1
        np = b2Collision.ClipSegmentToLine(clipPoints2, clipPoints1, rf.sideNormal2, rf.sideOffset2, rf.i2);
    
        if (np < b2Settings.b2_maxManifoldPoints) {
            return;
        }
    
        // Now clipPoints2 contains the clipped points.
        if (primaryAxis.type == b2EPAxis.e_edgeA) {
            manifold.localNormal = rf.normal.Copy();
            manifold.localPoint = rf.v1.Copy();
        } else {
            manifold.localNormal = polygonB.m_normals[rf.i1].Copy();
            manifold.localPoint = polygonB.m_vertices[rf.i1].Copy();
        }
    
        var pointCount = 0;
        for (var i = 0; i < b2Settings.b2_maxManifoldPoints; ++i) {
            var separation;
        
            separation = b2Math.DotVV(rf.normal, b2Math.SubtractVV(clipPoints2[i].v, rf.v1));
        
            if (separation <= this.m_radius) {
                var cp = manifold.points[pointCount];
            
                if (primaryAxis.type == b2EPAxis.e_edgeA) {
                    cp.localPoint = b2Math.MulTXV(this.m_xf, clipPoints2[i].v);
                    cp.id = clipPoints2[i].id.Copy();
                } else {
                    cp.localPoint = clipPoints2[i].v.Copy();
                    cp.id.cf.typeA = clipPoints2[i].id.cf.typeB;
                    cp.id.cf.typeB = clipPoints2[i].id.cf.typeA;
                    cp.id.cf.indexA = clipPoints2[i].id.cf.indexB;
                    cp.id.cf.indexB = clipPoints2[i].id.cf.indexA;
                }
            
                ++pointCount;
            }
        }
    
        manifold.pointCount = pointCount;
    }
    b2EPCollider.prototype.ComputeEdgeSeparation = function() {
        var axis = new b2EPAxis();
        axis.type = b2EPAxis.e_edgeA;
        axis.index = this.m_front ? 0 : 1;
        axis.separation = b2Settings.b2_maxFloat;

        for (var i = 0; i < this.m_polygonB.count; ++i) {
            var s = b2Math.DotVV(this.m_normal, b2Math.SubtractVV(this.m_polygonB.vertices[i], this.m_v1));
            if (s < axis.separation) {
                axis.separation = s;
            }
        }
        return axis;
    }
    b2EPCollider.prototype.ComputePolygonSeparation = function() {
        var axis = new b2EPAxis();
        axis.type = b2EPAxis.e_unknown;
        axis.index = -1;
        axis.separation = -b2Settings.b2_maxFloat;

        var perp = new b2Vec2(-this.m_normal.y, this.m_normal.x);
        for (var i = 0; i < this.m_polygonB.count; ++i) {
            var n = this.m_polygonB.normals[i].GetNegative();
            var s1 = b2Math.DotVV(n, b2Math.SubtractVV(this.m_polygonB.vertices[i], this.m_v1));
            var s2 = b2Math.DotVV(n, b2Math.SubtractVV(this.m_polygonB.vertices[i], this.m_v2));
            var s = b2Math.Min(s1, s2);
            if (s > this.m_radius) {
                axis.type = b2EPAxis.e_edgeB;
                axis.index = i;
                axis.separation = s;
                return axis;
            }
            // Adjacency
            if (b2Math.DotVV(n, perp) >= 0.0) {
                if (b2Math.DotVV(b2Math.SubtractVV(n, this.m_upperLimit), this.m_normal) < -b2Settings.b2_angularSlop) {
                    continue;
                }
            } else {
                if (b2Math.DotVV(b2Math.SubtractVV(n, this.m_lowerLimit), this.m_normal) < -b2Settings.b2_angularSlop) {
                    continue;
                }
            }

            if (s > axis.separation) {
                axis.type = b2EPAxis.e_edgeB;
                axis.index = i;
                axis.separation = s;
            }
        }
        return axis;
    }
    Box2D.postDefs.push(function() {
        b2EPCollider.e_isolated = 0;
        b2EPCollider.e_concave = 1;
        b2EPCollider.e_convex = 2;
    });
    b2Collision.CollideEdgeAndPolygon = function(manifold, edgeA, xfA, polygonB, xfB) {
        var collider = new b2EPCollider();
        collider.Collide(manifold, edgeA, xfA, polygonB, xfB);
    }
    b2Collision.TestOverlap = function(a, b) {
        var t1 = b.lowerBound;
        var t2 = a.upperBound;
        var d1X = t1.x - t2.x;
        var d1Y = t1.y - t2.y;
        t1 = a.lowerBound;
        t2 = b.upperBound;
        var d2X = t1.x - t2.x;
        var d2Y = t1.y - t2.y;
        if (d1X > 0.0 || d1Y > 0.0) return false;
        if (d2X > 0.0 || d2Y > 0.0) return false;
        return true;
    }
    b2Collision.TestOverlap2 = function(shapeA, indexA, shapeB, indexB, xfA, xfB) {
        var input = new b2DistanceInput();
        input.proxyA.Set(shapeA, indexA);
        input.proxyB.Set(shapeB, indexB);
        input.transformA = xfA.Copy();
        input.transformB = xfB.Copy();
        input.useRadii = true;

        var cache = new b2SimplexCache();
        cache.count = 0;

        var output = new b2DistanceOutput();

        b2Distance.Distance(output, cache, input);

        return output.distance < 10.0 * b2Settings.b2_epsilon;
    }
    b2Collision.b2TimeOfImpact = function(output, input) {
        ++b2Collision.b2_toiCalls;

        output.state = b2TOIOutput.e_unknown;
        output.t = input.tMax;
        var proxyA = input.proxyA;
        var proxyB = input.proxyB;
        var sweepA = input.sweepA.Copy();
        var sweepB = input.sweepB.Copy();

        // Large rotations can make the root finder fail, so we normalize the
        // sweep angles.
        sweepA.Normalize();
        sweepB.Normalize();

        var tMax = input.tMax;
        var totalRadius = proxyA.m_radius + proxyB.m_radius;
        var target = b2Math.Max(b2Settings.b2_linearSlop, totalRadius - 3.0 * b2Settings.b2_linearSlop);
        var tolerance = 0.25 * b2Settings.b2_linearSlop;
        b2Settings.b2Assert(target > tolerance);

        var t1 = 0.0;
        var k_maxIterations = 20;   // TODO_ERIN b2Settings
        var iter = 0;

        // Prepare input for distance query.
        var cache = b2Collision.s_cache;
        cache.count = 0;
        var distanceInput = b2Collision.s_distanceInput;
        distanceInput.proxyA = input.proxyA;
        distanceInput.proxyB = input.proxyB;
        distanceInput.useRadii = false;

        // The outer loop progressively attempts to compute new separating axes.
        // This loop terminates when an axis is repeated (no progress is made).
        for(;;) {
            var xfA = b2Collision.s_xfA, xfB = b2Collision.s_xfB;
            sweepA.GetTransform(xfA, t1);
            sweepB.GetTransform(xfB, t1);

            // Get the distance between shapes. We can also use the results
            // to get a separating axis.
            distanceInput.transformA = xfA;
            distanceInput.transformB = xfB;
            var distanceOutput = b2Collision.s_distanceOutput;
            b2Distance(distanceOutput, cache, distanceInput);

            // If the shapes are overlapped, we give up on continuous collision.
            if (distanceOutput.distance <= 0.0) {
                // Failure!
                output.state = b2TOIOutput.e_overlapped;
                output.t = 0.0;
                break;
            }

            if (distanceOutput.distance < target + tolerance) {
                // Victory!
                output.state = b2TOIOutput.e_touching;
                output.t = t1;
                break;
            }

            // Initialize the separating axis.
            var fcn = b2Collision.s_fcn;
            fcn.Initialize(cache, proxyA, sweepA, proxyB, sweepB, t1);

            // Compute the TOI on the separating axis. We do this by successively
            // resolving the deepest point. This loop is bounded by the number of vertices.
            var done = false;
            var t2 = tMax;
            var pushBackIter = 0;
            for (;;) {
                // Find the deepest point at t2. Store the witness point indices.
                var indexA0 = new Vector_a2j_Number(1);
                var indexB0 = new Vector_a2j_Number(1);
                var s2 = fcn.FindMinSeparation(indexA0, indexB0, t2);

                // Is the final configuration separated?
                if (s2 > target + tolerance) {
                    // Victory!
                    output.state = b2TOIOutput.e_separated;
                    output.t = tMax;
                    done = true;
                    break;
                }

                // Has the separation reached tolerance?
                if (s2 > target - tolerance) {
                    // Advance the sweeps
                    t1 = t2;
                    break;
                }

                // Compute the initial separation of the witness points.
                var s1 = fcn.Evaluate(indexA0[0], indexB0[0], t1);

                // Check for initial overlap. This might happen if the root finder
                // runs out of iterations.
                if (s1 < target - tolerance) {
                    output.state = b2TOIOutput.e_failed;
                    output.t = t1;
                    done = true;
                    break;
                }

                // Check for touching
                if (s1 <= target + tolerance) {
                    // Victory! t1 should hold the TOI (could be 0.0).
                    output.state = b2TOIOutput.e_touching;
                    output.t = t1;
                    done = true;
                    break;
                }

                // Compute 1D root of: f(x) - target = 0
                var rootIterCount = 0;
                var a1 = t1, a2 = t2;
                for (;;) {
                    // Use a mix of the secant rule and bisection.
                    var t;
                    if (rootIterCount & 1) {
                        // Secant rule to improve convergence.
                        t = a1 + (target - s1) * (a2 - a1) / (s2 - s1);
                    } else {
                        // Bisection to guarantee progress.
                        t = 0.5 * (a1 + a2);
                    }

                    var s = fcn.Evaluate(indexA0[0], indexB0[0], t);

                    if (b2Math.Abs(s - target) < tolerance) {
                        // t2 holds a tentative value for t1
                        t2 = t;
                        break;
                    }

                    // Ensure we continue to bracket the root.
                    if (s > target) {
                        a1 = t;
                        s1 = s;
                    } else {
                        a2 = t;
                        s2 = s;
                    }

                    ++rootIterCount;
                    ++b2Collision.b2_toiRootIters;

                    if (rootIterCount == 50) {
                        break;
                    }
                }

                b2Collision.b2_toiMaxRootIters = b2Math.Max(b2Collision.b2_toiMaxRootIters, rootIterCount);

                ++pushBackIter;

                if (pushBackIter == b2Settings.b2_maxPolygonVertices) {
                    break;
                }
            }

            ++iter;
            ++b2Collision.b2_toiIters;

            if (done) {
                break;
            }

            if (iter == k_maxIterations) {
                // Root finder got stuck. Semi-victory.
                output.state = b2TOIOutput.e_failed;
                output.t = t1;
                break;
            }
        }

        b2Collision.b2_toiMaxIters = b2Math.Max(b2Collision.b2_toiMaxIters, iter);
    }
    Box2D.postDefs.push(function() {
        Box2D.Collision.b2Collision.s_incidentEdge = b2Collision.MakeClipPointVector();
        Box2D.Collision.b2Collision.s_clipPoints1 = b2Collision.MakeClipPointVector();
        Box2D.Collision.b2Collision.s_clipPoints2 = b2Collision.MakeClipPointVector();
        Box2D.Collision.b2Collision.s_edgeAO = new Vector_a2j_Number(1);
        Box2D.Collision.b2Collision.s_edgeBO = new Vector_a2j_Number(1);
        Box2D.Collision.b2Collision.s_localTangent = new b2Vec2();
        Box2D.Collision.b2Collision.s_localNormal = new b2Vec2();
        Box2D.Collision.b2Collision.s_planePoint = new b2Vec2();
        Box2D.Collision.b2Collision.s_normal = new b2Vec2();
        Box2D.Collision.b2Collision.s_tangent = new b2Vec2();
        Box2D.Collision.b2Collision.s_tangent2 = new b2Vec2();
        Box2D.Collision.b2Collision.s_v11 = new b2Vec2();
        Box2D.Collision.b2Collision.s_v12 = new b2Vec2();
        Box2D.Collision.b2Collision.b2CollidePolyTempVec = new b2Vec2();
        Box2D.Collision.b2Collision.b2_nullFeature = 0x000000ff;

        Box2D.Collision.b2Collision.b2_toiCalls = 0;
        Box2D.Collision.b2Collision.b2_toiIters = 0;
        Box2D.Collision.b2Collision.b2_toiMaxIters = 0;
        Box2D.Collision.b2Collision.b2_toiRootIters = 0;
        Box2D.Collision.b2Collision.b2_toiMaxRootIters = 0;

        Box2D.Collision.b2Collision.s_cache = new b2SimplexCache();
        Box2D.Collision.b2Collision.s_distanceInput = new b2DistanceInput();
        Box2D.Collision.b2Collision.s_xfA = new b2Transform();
        Box2D.Collision.b2Collision.s_xfB = new b2Transform();
        Box2D.Collision.b2Collision.s_fcn = new b2SeparationFunction();
        Box2D.Collision.b2Collision.s_distanceOutput = new b2DistanceOutput();
    });
    b2ContactID.b2ContactID = function() {
        this.cf = new b2ContactFeature();
    };
    b2ContactID.prototype.b2ContactID = function() {
        this.cf._m_id = this;
        this._key = 0;
    }
    b2ContactID.prototype.Set = function(id) {
        this.key = id._key;
    }
    b2ContactID.prototype.Copy = function() {
        var id = new b2ContactID();
        id.key = this.key;
        return id;
    }
    Object.defineProperty(b2ContactID.prototype, 'key', {
        enumerable: false,
        configurable: true,
        get: function() {
            return this._key;
        }
    });
    Object.defineProperty(b2ContactID.prototype, 'key', {
        enumerable: false,
        configurable: true,
        set: function(value) {
            if (value === undefined) value = 0;
            this._key = value;
            this.cf._indexA = value & 0x000000ff;
            this.cf._indexB = ((value & 0x0000ff00) >> 8) & 0x000000ff;
            this.cf._typeA = ((value & 0x00ff0000) >> 16) & 0x000000ff;
            this.cf._typeB = ((value & 0xff000000) >> 24) & 0x000000ff;
        }
    });
    b2Distance.b2Distance = function() {};
    b2Distance.Distance = function(output, cache, input) {
        ++b2Distance.b2_gjkCalls;
        var proxyA = input.proxyA;
        var proxyB = input.proxyB;
        var transformA = input.transformA;
        var transformB = input.transformB;
        var simplex = b2Distance.s_simplex;
        simplex.ReadCache(cache, proxyA, transformA, proxyB, transformB);
        var vertices = simplex.m_vertices;
        var k_maxIters = 20;
        var saveA = b2Distance.s_saveA;
        var saveB = b2Distance.s_saveB;
        var saveCount = 0;
        var closestPoint = simplex.GetClosestPoint();
        var distanceSqr1 = closestPoint.LengthSquared();
        var distanceSqr2 = distanceSqr1;
        var i = 0;
        var p;
        var iter = 0;
        while (iter < k_maxIters) {
            saveCount = simplex.m_count;
            for (i = 0; i < saveCount; i++) {
                saveA[i] = vertices[i].indexA;
                saveB[i] = vertices[i].indexB;
            }
            switch (simplex.m_count) {
            case 1:
                break;
            case 2:
                simplex.Solve2();
                break;
            case 3:
                simplex.Solve3();
                break;
            default:
                b2Settings.b2Assert(false);
            }
            if (simplex.m_count == 3) {
                break;
            }
            p = simplex.GetClosestPoint();
            distanceSqr2 = p.LengthSquared();
            if (distanceSqr2 > distanceSqr1) {}
            distanceSqr1 = distanceSqr2;
            var d = simplex.GetSearchDirection();
            if (d.LengthSquared() < b2Settings.b2_epsilon * b2Settings.b2_epsilon) {
                break;
            }
            var vertex = vertices[simplex.m_count];
            vertex.indexA = proxyA.GetSupport(b2Math.MulTRV(transformA.q, d.GetNegative()));
            vertex.wA = b2Math.MulXV(transformA, proxyA.GetVertex(vertex.indexA));
            vertex.indexB = proxyB.GetSupport(b2Math.MulTRV(transformB.q, d));
            vertex.wB = b2Math.MulXV(transformB, proxyB.GetVertex(vertex.indexB));
            vertex.w = b2Math.SubtractVV(vertex.wB, vertex.wA);
            ++iter;
            ++b2Distance.b2_gjkIters;
            var duplicate = false;
            for (i = 0; i < saveCount; i++) {
                if (vertex.indexA == saveA[i] && vertex.indexB == saveB[i]) {
                    duplicate = true;
                    break;
                }
            }
            if (duplicate) {
                break;
            }++simplex.m_count;
        }
        b2Distance.b2_gjkMaxIters = b2Math.Max(b2Distance.b2_gjkMaxIters, iter);

        simplex.GetWitnessPoints(output.pointA, output.pointB);
        output.distance = b2Math.Distance(output.pointA, output.pointB);
        output.iterations = iter;
        simplex.WriteCache(cache);
        if (input.useRadii) {
            var rA = proxyA.m_radius;
            var rB = proxyB.m_radius;
            if (output.distance > rA + rB && output.distance > b2Settings.b2_epsilon) {
                output.distance -= rA + rB;
                var normal = b2Math.SubtractVV(output.pointB, output.pointA);
                normal.Normalize();
                output.pointA.x += rA * normal.x;
                output.pointA.y += rA * normal.y;
                output.pointB.x -= rB * normal.x;
                output.pointB.y -= rB * normal.y;
            } else {
                p = new b2Vec2();
                p.x = 0.5 * (output.pointA.x + output.pointB.x);
                p.y = 0.5 * (output.pointA.y + output.pointB.y);
                output.pointA.x = output.pointB.x = p.x;
                output.pointA.y = output.pointB.y = p.y;
                output.distance = 0.0;
            }
        }
    }
    Box2D.postDefs.push(function() {
        Box2D.Collision.b2Distance.s_simplex = new b2Simplex();
        Box2D.Collision.b2Distance.s_saveA = new Vector_a2j_Number(3);
        Box2D.Collision.b2Distance.s_saveB = new Vector_a2j_Number(3);
    });
    b2DistanceInput.b2DistanceInput = function() {
        this.proxyA = new b2DistanceProxy();
        this.proxyB = new b2DistanceProxy();
        this.transformA = new b2Transform();
        this.transformB = new b2Transform();
        this.useRadii = false;
    };
    b2DistanceOutput.b2DistanceOutput = function() {
        this.distance = 0.0;
        this.iterations = 0;
        this.pointA = new b2Vec2();
        this.pointB = new b2Vec2();
    };
    b2DistanceProxy.b2DistanceProxy = function() {
        this.m_buffer = new Vector(2);
        this.m_buffer[0] = new b2Vec2();
        this.m_buffer[1] = new b2Vec2();
        this.m_vertices = new Vector();
        this.m_count = 0;
        this.m_radius = 0.0;
    };
    b2DistanceProxy.prototype.Copy = function() {
        var copy = new b2DistanceProxy();
        copy.m_buffer[0].SetV(this.m_buffer[0]);
        copy.m_buffer[1].SetV(this.m_buffer[1]);
        copy.m_vertices = this.m_vertices;
        copy.m_count = this.m_count;
        copy.m_radius = this.m_radius;
        return copy;
    }
    b2DistanceProxy.prototype.Set = function(shape, index) {
        switch (shape.GetType()) {
        case b2Shape.e_circle:
            {
                var circle = (shape instanceof b2CircleShape ? shape : null);
                this.m_vertices = new Vector(1, true);
                this.m_vertices[0] = circle.m_p;
                this.m_count = 1;
                this.m_radius = circle.m_radius;
            }
            break;
        case b2Shape.e_polygon:
            {
                var polygon = (shape instanceof b2PolygonShape ? shape : null);
                this.m_vertices = polygon.m_vertices;
                this.m_count = polygon.m_vertexCount;
                this.m_radius = polygon.m_radius;
            }
            break;
        case b2Shape.e_chain:
            {
                var chain = (shape instanceof b2ChainShape ? shape : null);
                b2Settings.b2Assert(index >= 0 && index < chain.m_count);
                this.m_buffer[0].SetV(chain.m_vertices[index]);
                if (index + 1 < chain.m_count) {
                    this.m_buffer[1].SetV(chain.m_vertices[index + 1]);
                } else {
                    m_buffer[1].SetV(chain.m_vertices[0]);
                }
                this.m_vertices = this.m_buffer;
                this.m_count = 2;
                this.m_radius = chain.m_radius;
            }
            break;
        case b2Shape.e_edge:
            {
                var edge = (shape instanceof b2EdgeShape ? shape : null);
                this.m_vertices[0] = edge.m_vertex1;
                this.m_vertices[1] = edge.m_vertex2;
                this.m_count = 2;
                this.m_radius = edge.m_radius;
            }
            break;
        default:
            b2Settings.b2Assert(false);
        }
    }
    b2DistanceProxy.prototype.GetSupport = function(d) {
        var bestIndex = 0;
        var bestValue = this.m_vertices[0].x * d.x + this.m_vertices[0].y * d.y;
        for (var i = 1; i < this.m_count; ++i) {
            var value = this.m_vertices[i].x * d.x + this.m_vertices[i].y * d.y;
            if (value > bestValue) {
                bestIndex = i;
                bestValue = value;
            }
        }
        return bestIndex;
    }
    b2DistanceProxy.prototype.GetSupportVertex = function(d) {
        var bestIndex = 0;
        var bestValue = this.m_vertices[0].x * d.x + this.m_vertices[0].y * d.y;
        for (var i = 1; i < this.m_count; ++i) {
            var value = this.m_vertices[i].x * d.x + this.m_vertices[i].y * d.y;
            if (value > bestValue) {
                bestIndex = i;
                bestValue = value;
            }
        }
        return this.m_vertices[bestIndex];
    }
    b2DistanceProxy.prototype.GetVertexCount = function() {
        return this.m_count;
    }
    b2DistanceProxy.prototype.GetVertex = function(index) {
        b2Settings.b2Assert(0 <= index && index < this.m_count);
        return this.m_vertices[index];
    }
    b2DynamicTree.b2DynamicTree = function() {};
    b2DynamicTree.prototype.b2DynamicTree = function() {
        this.m_root = null;
        this.m_freeList = null;
        this.m_path = 0;
        this.m_insertionCount = 0;
    }
    b2DynamicTree.prototype.CreateProxy = function(aabb, userData) {
        var node = this.AllocateNode();
        var extendX = b2Settings.b2_aabbExtension;
        var extendY = b2Settings.b2_aabbExtension;
        node.aabb.lowerBound.x = aabb.lowerBound.x - extendX;
        node.aabb.lowerBound.y = aabb.lowerBound.y - extendY;
        node.aabb.upperBound.x = aabb.upperBound.x + extendX;
        node.aabb.upperBound.y = aabb.upperBound.y + extendY;
        node.userData = userData;
        node.height = 0;
        this.InsertLeaf(node);
        return node;
    }
    b2DynamicTree.prototype.DestroyProxy = function(proxy) {
        this.RemoveLeaf(proxy);
        this.FreeNode(proxy);
    }
    b2DynamicTree.prototype.MoveProxy = function(proxy, aabb, displacement) {
        b2Settings.b2Assert(proxy.IsLeaf());
        if (proxy.aabb.Contains(aabb)) {
            return false;
        }
        this.RemoveLeaf(proxy);
        var extendX = b2Settings.b2_aabbExtension + b2Settings.b2_aabbMultiplier * (displacement.x > 0 ? displacement.x : (-displacement.x));
        var extendY = b2Settings.b2_aabbExtension + b2Settings.b2_aabbMultiplier * (displacement.y > 0 ? displacement.y : (-displacement.y));
        proxy.aabb.lowerBound.x = aabb.lowerBound.x - extendX;
        proxy.aabb.lowerBound.y = aabb.lowerBound.y - extendY;
        proxy.aabb.upperBound.x = aabb.upperBound.x + extendX;
        proxy.aabb.upperBound.y = aabb.upperBound.y + extendY;
        this.InsertLeaf(proxy);
        return true;
    }
    /*
    b2DynamicTree.prototype.Rebalance = function(iterations) {
        if (iterations === undefined) iterations = 0;
        if (this.m_root == null) return;
        for (var i = 0; i < iterations; i++) {
            var node = this.m_root;
            var bit = 0;
            while (node.IsLeaf() == false) {
                node = (this.m_path >> bit) & 1 ? node.child2 : node.child1;
                bit = (bit + 1) & 31;
            }++this.m_path;
            this.RemoveLeaf(node);
            this.InsertLeaf(node);
        }
    }
    */
    b2DynamicTree.prototype.GetFatAABB = function(proxy) {
        return proxy.aabb;
    }
    b2DynamicTree.prototype.GetHeight = function() {
        if (this.m_root == null) return 0;
        return this.m_root.height;
    }
    b2DynamicTree.prototype.GetMaxBalance = function() {
        var maxBalance = 0;
        var stack = new Vector();
        var count = 0;
        stack[count++] = this.m_root;
        while (count > 0) {
            var node = stack[--count];
            if (!node.IsLeaf()) {
                var balance = b2Math.Abs(node.child1.height - node.child2.height);
                maxBalance = b2Math.Max(maxBalance, balance);
                stack[count++] = node.child1;
                stack[count++] = node.child2;
            }
        }
        return maxBalance;
    }
    b2DynamicTree.prototype.GetAreaRatio = function() {
        if (this.m_root == null) return 0;
        var rootArea = this.m_root.aabb.GetPerimeter();
        var totalArea = 0.0;
        var stack = new Vector();
        var count = 0;
        stack[count++] = this.m_root;
        while (count > 0) {
            var node = stack[--count];
            totalArea += node.aabb.GetPerimeter();
            if (!node.IsLeaf()) {
                stack[count++] = node.child1;
                stack[count++] = node.child2;
            }
        }
        return totalArea / rootArea;
    }
    b2DynamicTree.prototype.GetUserData = function(proxy) {
        return proxy.userData;
    }
    b2DynamicTree.prototype.Query = function(callback, aabb) {
        if (this.m_root == null) return;
        var stack = new Vector();
        var count = 0;
        stack[count++] = this.m_root;
        while (count > 0) {
            var node = stack[--count];
            if (node.aabb.TestOverlap(aabb)) {
                if (node.IsLeaf()) {
                    var proceed = callback.QueryCallback(node);
                    if (!proceed) return;
                } else {
                    stack[count++] = node.child1;
                    stack[count++] = node.child2;
                }
            }
        }
    }
    b2DynamicTree.prototype.RayCast = function(callback, input) {
        if (this.m_root == null) return;
        var p1 = input.p1;
        var p2 = input.p2;
        var r = b2Math.SubtractVV(p1, p2);
        r.Normalize();
        var v = b2Math.CrossFV(1.0, r);
        var abs_v = b2Math.AbsV(v);
        var maxFraction = input.maxFraction;
        var segmentAABB = new b2AABB();
        var tX = 0;
        var tY = 0; {
            tX = p1.x + maxFraction * (p2.x - p1.x);
            tY = p1.y + maxFraction * (p2.y - p1.y);
            segmentAABB.lowerBound.x = Math.min(p1.x, tX);
            segmentAABB.lowerBound.y = Math.min(p1.y, tY);
            segmentAABB.upperBound.x = Math.max(p1.x, tX);
            segmentAABB.upperBound.y = Math.max(p1.y, tY);
        }
        var stack = new Vector();
        var count = 0;
        stack[count++] = this.m_root;
        while (count > 0) {
            var node = stack[--count];
            if (node.aabb.TestOverlap(segmentAABB) == false) {
                continue;
            }
            var c = node.aabb.GetCenter();
            var h = node.aabb.GetExtents();
            var separation = Math.abs(v.x * (p1.x - c.x) + v.y * (p1.y - c.y)) - abs_v.x * h.x - abs_v.y * h.y;
            if (separation > 0.0) continue;
            if (node.IsLeaf()) {
                var subInput = new b2RayCastInput();
                subInput.p1 = input.p1;
                subInput.p2 = input.p2;
                subInput.maxFraction = input.maxFraction;
                maxFraction = callback(subInput, node);
                if (maxFraction == 0.0) return;
                if (maxFraction > 0.0) {
                    tX = p1.x + maxFraction * (p2.x - p1.x);
                    tY = p1.y + maxFraction * (p2.y - p1.y);
                    segmentAABB.lowerBound.x = Math.min(p1.x, tX);
                    segmentAABB.lowerBound.y = Math.min(p1.y, tY);
                    segmentAABB.upperBound.x = Math.max(p1.x, tX);
                    segmentAABB.upperBound.y = Math.max(p1.y, tY);
                }
            } else {
                stack[count++] = node.child1;
                stack[count++] = node.child2;
            }
        }
    }
    b2DynamicTree.prototype.AllocateNode = function() {
        var node;
        if (this.m_freeList) {
            node = this.m_freeList;
            this.m_freeList = node.parent;
            node.parent = null;
            node.child1 = null;
            node.child2 = null;
            node.height = 0;
            return node;
        }
        node = new b2TreeNode();
        node.height = 0;
        return node;
    }
    b2DynamicTree.prototype.FreeNode = function(node) {
        node.parent = this.m_freeList;
        node.height = -1;
        this.m_freeList = node;
    }
    b2DynamicTree.prototype.InsertLeaf = function(leaf) {
        ++this.m_insertionCount;
        if (this.m_root == null) {
            this.m_root = leaf;
            this.m_root.parent = null;
            return;
        }
        var center = leaf.aabb.GetCenter();
        var sibling = this.m_root;
        if (sibling.IsLeaf() == false) {
            do {
                var child1 = sibling.child1;
                var child2 = sibling.child2;
                var norm1 = Math.abs((child1.aabb.lowerBound.x + child1.aabb.upperBound.x) / 2 - center.x) + Math.abs((child1.aabb.lowerBound.y + child1.aabb.upperBound.y) / 2 - center.y);
                var norm2 = Math.abs((child2.aabb.lowerBound.x + child2.aabb.upperBound.x) / 2 - center.x) + Math.abs((child2.aabb.lowerBound.y + child2.aabb.upperBound.y) / 2 - center.y);
                if (norm1 < norm2) {
                    sibling = child1;
                } else {
                    sibling = child2;
                }
            }
            while (sibling.IsLeaf() == false)
        }
        var node1 = sibling.parent;
        var node2 = this.AllocateNode();
        node2.parent = node1;
        node2.userData = null;
        node2.aabb.Combine2(leaf.aabb, sibling.aabb);
        node2.height = sibling.height + 1;
        if (node1) {
            if (sibling.parent.child1 == sibling) {
                node1.child1 = node2;
            } else {
                node1.child2 = node2;
            }
            node2.child1 = sibling;
            node2.child2 = leaf;
            sibling.parent = node2;
            leaf.parent = node2;
            do {
                if (node1.aabb.Contains(node2.aabb)) break;
                node1.height = 1 + b2Math.Max(node1.child1.height, node1.child2.height);
                node1.aabb.Combine2(node1.child1.aabb, node1.child2.aabb);
                node2 = node1;
                node1 = node1.parent;
            }
            while (node1)
        } else {
            node2.child1 = sibling;
            node2.child2 = leaf;
            sibling.parent = node2;
            leaf.parent = node2;
            this.m_root = node2;
        }
    }
    b2DynamicTree.prototype.RemoveLeaf = function(leaf) {
        if (leaf == this.m_root) {
            this.m_root = null;
            return;
        }
        var node2 = leaf.parent;
        var node1 = node2.parent;
        var sibling;
        if (node2.child1 == leaf) {
            sibling = node2.child2;
        } else {
            sibling = node2.child1;
        }
        if (node1) {
            if (node1.child1 == node2) {
                node1.child1 = sibling;
            } else {
                node1.child2 = sibling;
            }
            sibling.parent = node1;
            this.FreeNode(node2);
            while (node1) {
                var oldAABB = node1.aabb;
                node1.aabb.Combine2(node1.child1.aabb, node1.child2.aabb);
                node1.height = 1 + b2Math.Max(node1.child1.height, node1.child2.height);
                if (oldAABB.Contains(node1.aabb)) break;
                node1 = node1.parent;
            }
        } else {
            this.m_root = sibling;
            sibling.parent = null;
            this.FreeNode(node2);
        }
    }
    b2DynamicTree.prototype.Validate = function() {}
    b2BroadPhase.b2BroadPhase = function() {
        this.m_tree = new b2DynamicTree();

        this.m_proxyCount = 0;

        this.m_pairCount = 0;
        this.m_pairBuffer = new Vector();
        this.m_moveCount = 0;
        this.m_moveBuffer = new Vector();
    };
    b2BroadPhase.prototype.CreateProxy = function(aabb, userData) {
        var proxy = this.m_tree.CreateProxy(aabb, userData);
        ++this.m_proxyCount;
        this.BufferMove(proxy);
        return proxy;
    }
    b2BroadPhase.prototype.DestroyProxy = function(proxy) {
        this.UnBufferMove(proxy);
        --this.m_proxyCount;
        this.m_tree.DestroyProxy(proxy);
    }
    b2BroadPhase.prototype.MoveProxy = function(proxy, aabb, displacement) {
        var buffer = this.m_tree.MoveProxy(proxy, aabb, displacement);
        if (buffer) {
            this.BufferMove(proxy);
        }
    }
    b2BroadPhase.prototype.TouchProxy = function(proxy) {
        this.BufferMove(proxy);
    }
    b2BroadPhase.prototype.TestOverlap = function(proxyA, proxyB) {
        var aabbA = this.m_tree.GetFatAABB(proxyA);
        var aabbB = this.m_tree.GetFatAABB(proxyB);
        return aabbA.TestOverlap(aabbB);
    }
    b2BroadPhase.prototype.GetUserData = function(proxy) {
        return this.m_tree.GetUserData(proxy);
    }
    b2BroadPhase.prototype.GetFatAABB = function(proxy) {
        return this.m_tree.GetFatAABB(proxy);
    }
    b2BroadPhase.prototype.GetProxyCount = function() {
        return this.m_proxyCount;
    }
    b2BroadPhase.prototype.GetTreeHeight = function() {
        return this.m_tree.GetHeight();
    }
    b2BroadPhase.prototype.GetTreeBalance = function() {
        return this.m_tree.GetMaxBalance();
    }
    b2BroadPhase.prototype.GetTreeQuality = function() {
        return this.m_tree.GetAreaRatio();
    }
    b2BroadPhase.prototype.QueryCallback = function(proxy) {
        var __this = this;
        if (proxy == __this.m_queryProxyId) return true;
        if (__this.m_pairCount == __this.m_pairBuffer.length) {
            __this.m_pairBuffer[__this.m_pairCount] = new b2Pair();
        }
        var pair = __this.m_pairBuffer[__this.m_pairCount];
        pair.proxyIdA = proxy < __this.m_queryProxyId ? proxy : __this.m_queryProxyId;
        pair.proxyIdB = proxy >= __this.m_queryProxyId ? proxy : __this.m_queryProxyId;
        ++__this.m_pairCount;
        return true;
    };
    b2BroadPhase.prototype.UpdatePairs = function(callback) {
        var __this = this;
        __this.m_pairCount = 0;
        for (var i = 0; i < __this.m_moveCount; ++i) {
            __this.m_queryProxyId = __this.m_moveBuffer[i];
            if (!__this.m_queryProxyId) continue;

            var fatAABB = __this.m_tree.GetFatAABB(__this.m_queryProxyId);
            __this.m_tree.Query(__this, fatAABB);
        }
        __this.m_moveCount = 0;

        // Sort the pair buffer to expose duplicates.
//        std::sort(this.m_pairBuffer, this.m_pairBuffer + this.m_pairCount, b2PairLessThan);
//        __this.m_pairBuffer.sort(this.PairLessThan);

        for (var i = 0; i < __this.m_pairCount;) {
            var primaryPair = __this.m_pairBuffer[i];
            var userDataA = __this.m_tree.GetUserData(primaryPair.proxyIdA);
            var userDataB = __this.m_tree.GetUserData(primaryPair.proxyIdB);
            callback.AddPair(userDataA, userDataB);
            ++i;
            while (i < __this.m_pairCount) {
                var pair = __this.m_pairBuffer[i];
                if (pair.proxyIdA != primaryPair.proxyIdA || pair.proxyIdB != primaryPair.proxyIdB) {
                    break;
                }
                ++i;
            }
        }
    }
    b2BroadPhase.prototype.Query = function(callback, aabb) {
        this.m_tree.Query(callback, aabb);
    }
    b2BroadPhase.prototype.RayCast = function(callback, input) {
        this.m_tree.RayCast(callback, input);
    }
    /*b2BroadPhase.prototype.Validate = function() {}
    b2BroadPhase.prototype.Rebalance = function(iterations) {
        if (iterations === undefined) iterations = 0;
        this.m_tree.Rebalance(iterations);
    }*/
    b2BroadPhase.prototype.BufferMove = function(proxy) {
        this.m_moveBuffer[this.m_moveCount] = proxy;
        ++this.m_moveCount;
    }
    b2BroadPhase.prototype.UnBufferMove = function(proxy) {
        var i = parseInt(this.m_moveBuffer.indexOf(proxy));
        this.m_moveBuffer.splice(i, 1);
        --this.m_moveCount;
    }
    b2BroadPhase.prototype.PairLessThan = function(pair1, pair2) {
        if (pair1.proxyIdA < pair2.proxyIdA) {
            return true;
        }
        if (pair1.proxyIdA == pair2.proxyIdA) {
            return pair1.proxyIdB < pair2.proxyIdB;
        }
        return false;
    }
    b2BroadPhase.__implements = {};
    b2BroadPhase.__implements[IBroadPhase] = true;
    b2TreeNode.b2TreeNode = function() {
        this.aabb = new b2AABB();
        this.height = -1;
    };
    b2TreeNode.prototype.IsLeaf = function() {
        return this.child1 == null;
    }
    b2Pair.b2Pair = function() {
        this.proxyIdA = null;
        this.proxyIdB = null;
        this.next = null;
    };
    b2Manifold.b2Manifold = function() {
        this.pointCount = 0;
    };
    b2Manifold.prototype.b2Manifold = function() {
        this.points = new Vector(b2Settings.b2_maxManifoldPoints);
        for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
            this.points[i] = new b2ManifoldPoint();
        }
        this.localNormal = new b2Vec2();
        this.localPoint = new b2Vec2();
    }
    b2Manifold.prototype.Reset = function() {
        for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
            ((this.points[i] instanceof b2ManifoldPoint ? this.points[i] : null)).Reset();
        }
        this.localNormal.SetZero();
        this.localPoint.SetZero();
        this.type = 0;
        this.pointCount = 0;
    }
    b2Manifold.prototype.Set = function(m) {
        this.pointCount = m.pointCount;
        for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
            this.points[i].Set(m.points[i]);
        }
        this.localNormal.SetV(m.localNormal);
        this.localPoint.SetV(m.localPoint);
        this.type = m.type;
    }
    b2Manifold.prototype.Copy = function() {
        var copy = new b2Manifold();
        copy.Set(this);
        return copy;
    }
    Box2D.postDefs.push(function() {
        Box2D.Collision.b2Manifold.e_circles = 0;
        Box2D.Collision.b2Manifold.e_faceA = 1;
        Box2D.Collision.b2Manifold.e_faceB = 2;
    });
    b2ManifoldPoint.b2ManifoldPoint = function() {
        this.localPoint = new b2Vec2();
        this.id = new b2ContactID();
    };
    b2ManifoldPoint.prototype.b2ManifoldPoint = function() {
        this.Reset();
    }
    b2ManifoldPoint.prototype.Reset = function() {
        this.localPoint.SetZero();
        this.normalImpulse = 0.0;
        this.tangentImpulse = 0.0;
        this.id.key = 0;
    }
    b2ManifoldPoint.prototype.Set = function(m) {
        this.localPoint.SetV(m.localPoint);
        this.normalImpulse = m.normalImpulse;
        this.tangentImpulse = m.tangentImpulse;
        this.id.Set(m.id);
    }
    b2RayCastInput.b2RayCastInput = function() {
        this.p1 = new b2Vec2();
        this.p2 = new b2Vec2();
        this.maxFraction = 0.0;
    };
    b2RayCastOutput.b2RayCastOutput = function() {
        this.normal = new b2Vec2();
        this.fraction = 0.0;
    };
    b2Rope.b2Rope = function() {
        this.m_gravity = new b2Vec2();
    };
    b2Rope.prototype.Initialize = function(def) {
        b2Settings.b2Assert(def.count >= 3);
        this.m_count = def.count;
        this.m_ps = new Vector(this.m_count);
        this.m_p0s = new Vector(this.m_count);
        this.m_vs = new Vector(this.m_count);
        this.m_ims = new Vector(this.m_count);
        for (var i = 0; i < this.m_count; i++) {
            this.m_ps[i] = new b2Vec2();
            this.m_ps[i].SetV(def.vertices[i]);
            this.m_p0s[i] = new b2Vec2();
            this.m_p0s[i].SetV(def.vertices[i]);
            this.m_vs[i] = new b2Vec2();
            this.m_vs[i].SetZero();

            var m = def.masses[i];
            if (m > 0.0) {
                this.m_ims[i] = 1.0 / m;
            } else {
                this.m_ims[i] = 0.0;
            }
        }
        var count2 = this.m_count -1;
        var count3 = this.m_count -2;
        this.m_Ls = new Vector(count2);
        this.m_as = new Vector(count3);
        for (var i = 0; i < count2; i++) {
            var p1 = this.m_ps[i];
            var p2 = this.m_ps[i + 1];
            this.m_Ls[i] = b2Math.Distance(p1, p2);
        }
        for (var i = 0; i < count3; i++) {
            var p1 = this.m_ps[i];
            var p2 = this.m_ps[i + 1];
            var p3 = this.m_ps[i + 2];

            var d1 = new b2Vec2(p2.x - p1.x, p2.y - p1.y);
            var d2 = new b2Vec2(p3.x - p2.x, p3.y - p2.y);
            var a = b2Math.CrossVV(d1, d2);
            var b = b2Math.DotVV(d1, d2);
            this.m_as[i] = Math.atan2(a, b);
        }
        this.m_gravity.SetV(def.gravity);
        this.m_damping = def.damping;
        this.m_k2 = def.k2;
        this.m_k3 = def.k3;
    }
    b2Rope.prototype.Step = function(h, iterations) {
        if (h == 0.0) {
            return;
        }
        var d = Math.exp(- h * this.m_damping);
        for (var i = 0; i < this.m_count; i++) {
            this.m_p0s[i].SetV(this.m_ps[i]);
            if (this.m_ims[i] > 0.0) {
                this.m_vs[i].x += h * this.m_gravity.x;
                this.m_vs[i].y += h * this.m_gravity.y;
            }
            this.m_vs[i].Multiply(d);
            this.m_ps[i].x += h * this.m_vs[i].x;
            this.m_ps[i].y += h * this.m_vs[i].y;
        }
        for (var i = 0; i < iterations; i++) {
            this.SolveC2();
            this.SolveC3();
            this.SolveC2();
        }
        var inv_h = 1.0 / h;
        for (var i = 0; i < this.m_count; i++) {
            this.m_vs[i].x = inv_h * (this.m_ps[i].x - this.m_p0s[i].x);
            this.m_vs[i].y = inv_h * (this.m_ps[i].y - this.m_p0s[i].y);
        }
    }
    b2Rope.prototype.SolveC2 = function() {
        var count2 = this.m_count - 1;

        for (var i = 0; i < count2; ++i) {
            var p1 = this.m_ps[i];
            var p2 = this.m_ps[i + 1];

            var d = new b2Vec2(p2.x - p1.x, p2.y - p1.y);
            var L = d.Normalize();

            var im1 = this.m_ims[i];
            var im2 = this.m_ims[i + 1];

            if (im1 + im2 == 0.0) {
                continue;
            }

            var s1 = im1 / (im1 + im2);
            var s2 = im2 / (im1 + im2);

            p1.x -= this.m_k2 * s1 * (this.m_Ls[i] - L) * d.x;
            p1.y -= this.m_k2 * s1 * (this.m_Ls[i] - L) * d.y;
            p2.x += this.m_k2 * s2 * (this.m_Ls[i] - L) * d.x;
            p2.y += this.m_k2 * s2 * (this.m_Ls[i] - L) * d.y;

//            this.m_ps[i].SetV(p1);
//            this.m_ps[i + 1].SetV(p2);
        }
    }
    b2Rope.prototype.SolveC3 = function() {
        var count3 = this.m_count - 2;

        for (var i = 0; i < count3; ++i) {
            var p1 = this.m_ps[i];
            var p2 = this.m_ps[i + 1];
            var p3 = this.m_ps[i + 2];

            var m1 = this.m_ims[i];
            var m2 = this.m_ims[i + 1];
            var m3 = this.m_ims[i + 2];

            var d1 = new b2Vec2(p2.x - p1.x, p2.y - p1.y);
            var d2 = new b2Vec2(p3.x - p2.x, p3.y - p2.y);

            var L1sqr = d1.LengthSquared();
            var L2sqr = d2.LengthSquared();

            if (L1sqr * L2sqr == 0.0) {
                continue;
            }

            var a = b2Math.CrossVV(d1, d2);
            var b = b2Math.DotVV(d1, d2);

            var angle = Math.atan2(a, b);

            var Jd1 = d1.Skew(); Jd1.Multiply(-1.0 / L1sqr);
            var Jd2 = d2.Skew(); Jd2.Multiply(1.0 / L2sqr);

            var J1 = Jd1.GetNegative();
            var J2 = new b2Vec2(Jd1.x - Jd2.x, Jd1.y - Jd2.y);
            var J3 = new b2Vec2(Jd2.x, Jd2.y);

            var mass = m1 * b2Math.DotVV(J1, J1) + m2 * b2Math.DotVV(J2, J2) + m3 * b2Math.DotVV(J3, J3);
            if (mass == 0.0) {
                continue;
            }

            mass = 1.0 / mass;

            var C = angle - this.m_as[i];

            while (C > b2Settings.b2_pi) {
                angle -= 2 * b2_pi;
                C = angle - this.m_as[i];
            }

            while (C < -b2Settings.b2_pi) {
                angle += 2.0 * b2_pi;
                C = angle - this.m_as[i];
            }

            var impulse = - this.m_k3 * mass * C;

            p1.x += (m1 * impulse) * J1.x;
            p1.y += (m1 * impulse) * J1.y;
            p2.x += (m2 * impulse) * J2.x;
            p2.y += (m2 * impulse) * J2.y;
            p3.x += (m3 * impulse) * J3.x;
            p3.y += (m3 * impulse) * J3.y;

//            this.m_ps[i].SetV(p1);
//            this.m_ps[i + 1].SetV(p2);
//            this.m_ps[i + 2].SetV(p3);
        }
    }
    b2Rope.prototype.GetVertexCount = function() {
        return this.m_count;
    }
    b2Rope.prototype.GetVertices = function() {
        return this.m_ps;
    }
    b2Rope.prototype.Draw = function() {
    }
    b2Rope.prototype.SetAngle = function(angle) {
        var count3 = this.m_count - 2;
        for (var i = 0; i < count3; ++i) {
            this.m_as[i] = angle;
        }
    }
    b2Rope.prototype.b2Rope = function() {
        this.m_count = 0;
        this.m_ps = null;
        this.m_p0s = null;
        this.m_vs = null;
        this.m_ims = null;
        this.m_Ls = null;
        this.m_as = null;
        this.m_gravity.SetZero();
        this.m_k2 = 1.0;
        this.m_k3 = 0.1;
    }
    b2RopeDef.b2RopeDef = function() {
        this.masses = null;
        this.vertices = null;
        this.count = 0;
        this.gravity = new b2Vec2();
        this.damping = 0.1;
        this.k2 = 0.9;
        this.k3 = 0.1;
    };
    b2SeparationFunction.b2SeparationFunction = function() {
        this.m_sweepA = new b2Sweep();
        this.m_sweepB = new b2Sweep();
        this.m_localPoint = new b2Vec2();
        this.m_axis = new b2Vec2();
    };
    b2SeparationFunction.prototype.Initialize = function(cache, proxyA, sweepA, proxyB, sweepB, t1) {
        this.m_proxyA = proxyA;
        this.m_proxyB = proxyB;
        var count = parseInt(cache.count);
        b2Settings.b2Assert(0 < count && count < 3);

        this.m_sweepA.SetV(sweepA);
        this.m_sweepB.SetV(sweepB);

        var xfA = new b2Transform();
        var xfB = new b2Transform();
        this.m_sweepA.GetTransform(xfA, t1);
        this.m_sweepB.GetTransform(xfB, t1);

        if (count == 1) {
            this.m_type = b2SeparationFunction.e_points;
            var localPointA = this.m_proxyA.GetVertex(cache.indexA[0]);
            var localPointB = this.m_proxyB.GetVertex(cache.indexB[0]);
            var pointAX = xfA.p.x + xfA.q.c * localPointA.x - xfA.q.s * localPointA.y;
            var pointAY = xfA.p.y + xfA.q.s * localPointA.x + xfA.q.c * localPointA.y;
            var pointBX = xfB.p.x + xfB.q.c * localPointB.x - xfB.q.s * localPointB.y;
            var pointBY = xfB.p.y + xfB.q.s * localPointB.x + xfB.q.c * localPointB.y;
            this.m_axis.x = pointBX - pointAX;
            this.m_axis.y = pointBY - pointAY;
            s = this.m_axis.Normalize();
            return s;
        } else if (cache.indexA[0] == cache.indexA[1]) {
            this.m_type = b2SeparationFunction.e_faceB;
            var localPointB1 = this.m_proxyB.GetVertex(cache.indexB[0]);
            var localPointB2 = this.m_proxyB.GetVertex(cache.indexB[1]);
            this.m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointB2, localPointB1), 1.0);
            this.m_axis.Normalize();
            var normal = b2Math.MulRV(xfB.q, this.m_axis);

            this.m_localPoint.x = 0.5 * (localPointB1.x + localPointB2.x);
            this.m_localPoint.y = 0.5 * (localPointB1.y + localPointB2.y);
            var pointB = b2Math.MulXV(xfB, this.m_localPoint);

            var localPonitA = proxyA.GetVertex(cache.indexA[0]);
            var pointA = b2Math.MulXV(xfA, localPointA);

            s = b2Math.DotVV(b2Math.SubtractVV(pointA, pointB), normal);
            if (s < 0.0) {
                this.m_axis.SetV(this.m_axis.GetNegative());
                s = -s;
            }
            return s;
        } else {
            this.m_type = b2SeparationFunction.e_faceA;
            var localPointA1 = this.m_proxyB.GetVertex(cache.indexA[0]);
            var localPointA2 = this.m_proxyB.GetVertex(cache.indexA[1]);
            this.m_axis = b2Math.CrossVF(b2Math.SubtractVV(localPointA2, localPointA1), 1.0);
            this.m_axis.Normalize();
            var normal = b2Math.MulRV(xfA.q, this.m_axis);

            this.m_localPoint.x = 0.5 * (localPointA1.x + localPointA2.x);
            this.m_localPoint.y = 0.5 * (localPointA1.y + localPointA2.y);
            var pointA = b2Math.MulXV(xfA, this.m_localPoint);

            var localPonitB = this.m_proxyB.GetVertex(cache.indexB[0]);
            var pointB = b2Math.MulXV(xfB, localPointB);

            s = b2Math.DotVV(b2Math.SubtractVV(pointB, pointA), normal);
            if (s < 0.0) {
                this.m_axis.SetV(this.m_axis.GetNegative());
                s = -s;
            }
            return s;
        }
    }
    b2SeparationFunction.prototype.FindMinSeparation = function(indexA0, indexB0, t) {
        var xfA = new b2Transform;
        var xfB = new b2Transform;
        this.m_sweepA.GetTransform(xfA, t);
        this.m_sweepB.GetTransform(xfB, t);

        switch (this.m_type) {
        case b2SeparationFunction.e_points:
        {
            var axisA = b2Math.MulTRV(xfA.q, this.m_axis);
            var axisB = b2Math.MulTRV(xfB.q, this.m_axis.GetNegative());

            indexA0[0] = this.m_proxyA.GetSupport(axisA);
            indexB0[0] = this.m_proxyB.GetSupport(axisB);

            var localPointA = this.m_proxyA.GetVertex(indexA0[0]);
            var localPointB = this.m_proxyB.GetVertex(indexB0[0]);

            var pointA = b2Math.MulXV(xfA, localPointA);
            var pointB = b2Math.MulXV(xfB, localPointB);

            var separation = (pointB.x - pointA.x) * this.m_axis.x + (pointB.y - pointA.y) * this.m_axis.y;
            return separation;
        }

        case b2SeparationFunction.e_faceA:
        {
            var normal = b2Math.MulRV(xfA.q, this.m_axis);
            var pointA = b2Math.MulXV(xfA, this.m_localPoint);

            var axisB = b2Math.MulTRV(xfB.q, normal.GetNegative());

            indexA0[0] = -1;
            indexB0[0] = this.m_proxyB.GetSupport(axisB);

            var localPointB = this.m_proxyB.GetVertex(indexB0[0]);
            var pointB = b2Math.MulXV(xfB, localPointB);

            var separation = (pointB.x - pointA.x) * normal.x + (pointB.y - pointA.y) * normal.y;
            return separation;
        }

        case b2SeparationFunction.e_faceB:
        {
            var normal = b2Math.MulRV(xfB.q, this.m_axis);
            var pointA = b2Math.MulXV(xfB, this.m_localPoint);

            var axisA = b2Math.MulTRV(xfA.q, normal.GetNegative());

            indexB0[0] = -1;
            indexA0[0] = this.m_proxyB.GetSupport(axisA);

            var localPointA = this.m_proxyA.GetVertex(indexA0[0]);
            var pointA = b2Math.MulXV(xfA, localPointA);

            var separation = (pointB.x - pointA.x) * normal.x + (pointB.y - pointA.y) * normal.y;
            return separation;
        }

        default:
            b2Assert(false);
            indexA0[0] = -1;
            indexB0[0] = -1;
            return 0.0;
        }
    }
    b2SeparationFunction.prototype.Evaluate = function(indexA, indexB, t) {
        var xfA = new b2Transform;
        var xfB = new b2Transform;
        this.m_sweepA.GetTransform(xfA, t);
        this.m_sweepB.GetTransform(xfB, t);
        switch (this.m_type) {
        case b2SeparationFunction.e_points:
        {
            var axisA = b2Math.MulTRV(xfA.q, this.m_axis);
            var axisB = b2Math.MulTRV(xfB.q, this.m_axis.GetNegative());
            var localPointA = this.m_proxyA.GetSupportVertex(indexA);
            var localPointB = this.m_proxyB.GetSupportVertex(indexB);
            var pointA = b2Math.MulXV(xfA, localPointA);
            var pointB = b2Math.MulXV(xfB, localPointB);
            var seperation = b2Math.DotVV(b2Math.SubtractVV(pointB, pointA), this.m_axis);
            return seperation;
        }
        case b2SeparationFunction.e_faceA:
        {
            var normal = b2Math.MulRV(xfA.q, this.m_axis);
            var pointA = b2Math.MulXV(xfA, this.m_localPoint);
            var axisB = b2Math.MulTRV(xfB.q, normal.GetNegative());
            var localPointB = this.m_proxyB.GetSupportVertex(indexB);
            var pointB = b2Math.MulXV(xfB, localPointB);
            var seperation = b2Math.DotVV(b2Math.SubtractVV(pointB, pointA), normal);
            return seperation;
        }
        case b2SeparationFunction.e_faceB:
        {
            var normal = b2Math.MulRV(xfB.q, this.m_axis);
            var pointB = b2Math.MulXV(xfB, this.m_localPoint);
            var axisA = b2Math.MulTRV(xfA.q, normal.GetNegative());
            var localPointA = this.m_proxyA.GetSupportVertex(indexA);
            var pointA = b2Math.MulXV(xfA, localPointA);
            var seperation = b2Math.DotVV(b2Math.SubtractVV(pointA, pointB), normal);
            return seperation;
        }
        default:
            b2Settings.b2Assert(false);
            return 0.0;
        }
    }
    Box2D.postDefs.push(function() {
        Box2D.Collision.b2SeparationFunction.e_points = 0;
        Box2D.Collision.b2SeparationFunction.e_faceA = 1;
        Box2D.Collision.b2SeparationFunction.e_faceB = 2;
    });
    b2Simplex.b2Simplex = function() {
        this.m_v1 = new b2SimplexVertex();
        this.m_v2 = new b2SimplexVertex();
        this.m_v3 = new b2SimplexVertex();
        this.m_vertices = new Vector(3);
    };
    b2Simplex.prototype.b2Simplex = function() {
        this.m_vertices[0] = this.m_v1;
        this.m_vertices[1] = this.m_v2;
        this.m_vertices[2] = this.m_v3;
    }
    b2Simplex.prototype.ReadCache = function(cache, proxyA, transformA, proxyB, transformB) {
        b2Settings.b2Assert(0 <= cache.count && cache.count <= 3);
        var wALocal;
        var wBLocal;
        this.m_count = cache.count;
        var vertices = this.m_vertices;
        for (var i = 0; i < this.m_count; i++) {
            var v = vertices[i];
            v.indexA = cache.indexA[i];
            v.indexB = cache.indexB[i];
            wALocal = proxyA.GetVertex(v.indexA);
            wBLocal = proxyB.GetVertex(v.indexB);
            v.wA = b2Math.MulXV(transformA, wALocal);
            v.wB = b2Math.MulXV(transformB, wBLocal);
            v.w = b2Math.SubtractVV(v.wB, v.wA);
            v.a = 0;
        }
        if (this.m_count > 1) {
            var metric1 = cache.metric;
            var metric2 = this.GetMetric();
            if (metric2 < 0.5 * metric1 || 2.0 * metric1 < metric2 || metric2 < b2Settings.b2_epsilon) {
                this.m_count = 0;
            }
        }
        if (this.m_count == 0) {
            v = vertices[0];
            v.indexA = 0;
            v.indexB = 0;
            wALocal = proxyA.GetVertex(0);
            wBLocal = proxyB.GetVertex(0);
            v.wA = b2Math.MulXV(transformA, wALocal);
            v.wB = b2Math.MulXV(transformB, wBLocal);
            v.w = b2Math.SubtractVV(v.wB, v.wA);
            this.m_count = 1;
        }
    }
    b2Simplex.prototype.WriteCache = function(cache) {
        cache.metric = this.GetMetric();
        cache.count = Box2D.parseUInt(this.m_count);
        var vertices = this.m_vertices;
        for (var i = 0; i < this.m_count; i++) {
            cache.indexA[i] = Box2D.parseUInt(vertices[i].indexA);
            cache.indexB[i] = Box2D.parseUInt(vertices[i].indexB);
        }
    }
    b2Simplex.prototype.GetSearchDirection = function() {
        switch (this.m_count) {
        case 1:
            return this.m_v1.w.GetNegative();
        case 2:
            {
                var e12 = b2Math.SubtractVV(this.m_v2.w, this.m_v1.w);
                var sgn = b2Math.CrossVV(e12, this.m_v1.w.GetNegative());
                if (sgn > 0.0) {
                    return b2Math.CrossFV(1.0, e12);
                } else {
                    return b2Math.CrossVF(e12, 1.0);
                }
            }
        default:
            b2Settings.b2Assert(false);
            return new b2Vec2();
        }
    }
    b2Simplex.prototype.GetClosestPoint = function() {
        switch (this.m_count) {
        case 0:
            return new b2Vec2();
        case 1:
            return this.m_v1.w;
        case 2:
            return new b2Vec2(this.m_v1.a * this.m_v1.w.x + this.m_v2.a * this.m_v2.w.x, this.m_v1.a * this.m_v1.w.y + this.m_v2.a * this.m_v2.w.y);
        default:
            return new b2Vec2();
        }
    }
    b2Simplex.prototype.GetWitnessPoints = function(pA, pB) {
        switch (this.m_count) {
        case 0:
//            b2Settings.b2Assert(false);
            break;
        case 1:
            pA.SetV(this.m_v1.wA);
            pB.SetV(this.m_v1.wB);
            break;
        case 2:
            pA.x = this.m_v1.a * this.m_v1.wA.x + this.m_v2.a * this.m_v2.wA.x;
            pA.y = this.m_v1.a * this.m_v1.wA.y + this.m_v2.a * this.m_v2.wA.y;
            pB.x = this.m_v1.a * this.m_v1.wB.x + this.m_v2.a * this.m_v2.wB.x;
            pB.y = this.m_v1.a * this.m_v1.wB.y + this.m_v2.a * this.m_v2.wB.y;
            break;
        case 3:
            pB.x = pA.x = this.m_v1.a * this.m_v1.wA.x + this.m_v2.a * this.m_v2.wA.x + this.m_v3.a * this.m_v3.wA.x;
            pB.y = pA.y = this.m_v1.a * this.m_v1.wA.y + this.m_v2.a * this.m_v2.wA.y + this.m_v3.a * this.m_v3.wA.y;
            break;
        default:
//            b2Settings.b2Assert(false);
            break;
        }
    }
    b2Simplex.prototype.GetMetric = function() {
        switch (this.m_count) {
        case 0:
//            b2Settings.b2Assert(false);
            return 0.0;
        case 1:
            return 0.0;
        case 2:
            return b2Math.Distance(this.m_v1.w, this.m_v2.w);
        case 3:
            return b2Math.CrossVV(b2Math.SubtractVV(this.m_v2.w, this.m_v1.w), b2Math.SubtractVV(this.m_v3.w, this.m_v1.w));
        default:
//            b2Settings.b2Assert(false);
            return 0.0;
        }
    }
    b2Simplex.prototype.Solve2 = function() {
        var w1 = this.m_v1.w;
        var w2 = this.m_v2.w;
        var e12 = b2Math.SubtractVV(w2, w1);
        var d12_2 = (-(w1.x * e12.x + w1.y * e12.y));
        if (d12_2 <= 0.0) {
            this.m_v1.a = 1.0;
            this.m_count = 1;
            return;
        }
        var d12_1 = (w2.x * e12.x + w2.y * e12.y);
        if (d12_1 <= 0.0) {
            this.m_v2.a = 1.0;
            this.m_count = 1;
            this.m_v1.Set(this.m_v2);
            return;
        }
        var inv_d12 = 1.0 / (d12_1 + d12_2);
        this.m_v1.a = d12_1 * inv_d12;
        this.m_v2.a = d12_2 * inv_d12;
        this.m_count = 2;
    }
    b2Simplex.prototype.Solve3 = function() {
        var w1 = this.m_v1.w;
        var w2 = this.m_v2.w;
        var w3 = this.m_v3.w;
        var e12 = b2Math.SubtractVV(w2, w1);
        var w1e12 = b2Math.DotVV(w1, e12);
        var w2e12 = b2Math.DotVV(w2, e12);
        var d12_1 = w2e12;
        var d12_2 = (-w1e12);
        var e13 = b2Math.SubtractVV(w3, w1);
        var w1e13 = b2Math.DotVV(w1, e13);
        var w3e13 = b2Math.DotVV(w3, e13);
        var d13_1 = w3e13;
        var d13_2 = (-w1e13);
        var e23 = b2Math.SubtractVV(w3, w2);
        var w2e23 = b2Math.DotVV(w2, e23);
        var w3e23 = b2Math.DotVV(w3, e23);
        var d23_1 = w3e23;
        var d23_2 = (-w2e23);
        var n123 = b2Math.CrossVV(e12, e13);
        var d123_1 = n123 * b2Math.CrossVV(w2, w3);
        var d123_2 = n123 * b2Math.CrossVV(w3, w1);
        var d123_3 = n123 * b2Math.CrossVV(w1, w2);
        if (d12_2 <= 0.0 && d13_2 <= 0.0) {
            this.m_v1.a = 1.0;
            this.m_count = 1;
            return;
        }
        if (d12_1 > 0.0 && d12_2 > 0.0 && d123_3 <= 0.0) {
            var inv_d12 = 1.0 / (d12_1 + d12_2);
            this.m_v1.a = d12_1 * inv_d12;
            this.m_v2.a = d12_2 * inv_d12;
            this.m_count = 2;
            return;
        }
        if (d13_1 > 0.0 && d13_2 > 0.0 && d123_2 <= 0.0) {
            var inv_d13 = 1.0 / (d13_1 + d13_2);
            this.m_v1.a = d13_1 * inv_d13;
            this.m_v3.a = d13_2 * inv_d13;
            this.m_count = 2;
            this.m_v2.Set(this.m_v3);
            return;
        }
        if (d12_1 <= 0.0 && d23_2 <= 0.0) {
            this.m_v2.a = 1.0;
            this.m_count = 1;
            this.m_v1.Set(this.m_v2);
            return;
        }
        if (d13_1 <= 0.0 && d23_1 <= 0.0) {
            this.m_v3.a = 1.0;
            this.m_count = 1;
            this.m_v1.Set(this.m_v3);
            return;
        }
        if (d23_1 > 0.0 && d23_2 > 0.0 && d123_1 <= 0.0) {
            var inv_d23 = 1.0 / (d23_1 + d23_2);
            this.m_v2.a = d23_1 * inv_d23;
            this.m_v3.a = d23_2 * inv_d23;
            this.m_count = 2;
            this.m_v1.Set(this.m_v3);
            return;
        }
        var inv_d123 = 1.0 / (d123_1 + d123_2 + d123_3);
        this.m_v1.a = d123_1 * inv_d123;
        this.m_v2.a = d123_2 * inv_d123;
        this.m_v3.a = d123_3 * inv_d123;
        this.m_count = 3;
    }
    b2SimplexCache.b2SimplexCache = function() {
        this.indexA = new Vector_a2j_Number(3);
        this.indexB = new Vector_a2j_Number(3);
    };
    b2SimplexVertex.b2SimplexVertex = function() {};
    b2SimplexVertex.prototype.Set = function(other) {
        this.wA.SetV(other.wA);
        this.wB.SetV(other.wB);
        this.w.SetV(other.w);
        this.a = other.a;
        this.indexA = other.indexA;
        this.indexB = other.indexB;
    }
    b2SimplexVertex.prototype.Copy = function() {
        var copy = new b2SimplexVertex();
        copy.Set(this);
        return copy;
    }
    b2TOIInput.b2TOIInput = function() {
        this.proxyA = new b2DistanceProxy();
        this.proxyB = new b2DistanceProxy();
        this.sweepA = new b2Sweep();
        this.sweepB = new b2Sweep();
        this.tMax = 0;
    };
    b2TOIOutput.b2TOIOutput = function() {
    };
    Box2D.postDefs.push(function() {
        Box2D.Collision.b2TOIOutput.e_unknown = 0;
        Box2D.Collision.b2TOIOutput.e_failed = 1;
        Box2D.Collision.b2TOIOutput.e_overlapped = 2;
        Box2D.Collision.b2TOIOutput.e_touching = 3;
        Box2D.Collision.b2TOIOutput.e_separated = 4;
    });
    b2WorldManifold.b2WorldManifold = function() {
        this.normal = new b2Vec2();
    };
    b2WorldManifold.prototype.b2WorldManifold = function() {
        this.points = new Vector(b2Settings.b2_maxManifoldPoints);
        for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
            this.points[i] = new b2Vec2();
        }
    }
    b2WorldManifold.prototype.Initialize = function(manifold, xfA, radiusA, xfB, radiusB) {
        if (manifold.pointCount == 0) {
            return;
        }
        var tRot;
        var tVec;
        switch (manifold.type) {
        case b2Manifold.e_circles:
            {
                this.normal.Set(1.0, 0.0);
                var pointA = b2Math.MulXV(xfA, manifold.localPoint);
                var pointB = b2Math.MulXV(xfB, manifold.points[0].localPoint);
                if (b2Math.DistanceSquared(pointA, pointB) > b2Settings.b2_epsilon * b2Settings.b2_epsilon) {
                    this.normal = b2Math.SubtractVV(pointB, pointA);
                    this.normal.Normalize();
                }
                var cA = b2Math.AddVV(pointA, b2Math.MulFV(radiusA, this.normal));
                var cB = b2Math.SubtractVV(pointB, b2Math.MulFV(radiusB, this.normal));
                this.points[0].x = 0.5 * (cA.x + cB.x);
                this.points[0].y = 0.5 * (cA.y + cB.y);
            }
            break;
        case b2Manifold.e_faceA:
            {
                tRot = xfA.q;
                tVec = manifold.localNormal;
                this.normal.x = tRot.c * tVec.x - tRot.s * tVec.y;
                this.normal.y = tRot.s * tVec.x + tRot.c * tVec.y;
                tVec = manifold.localPoint;
                var planePointX = tRot.c * tVec.x - tRot.s * tVec.y + xfA.p.x;
                var planePointY = tRot.s * tVec.x + tRot.c * tVec.y + xfA.p.y;
                tRot = xfB.q;
                for (var i = 0; i < manifold.pointCount; ++i) {
                    tVec = manifold.points[i].localPoint;
                    var clipPointX = tRot.c * tVec.x - tRot.s * tVec.y + xfB.p.x;
                    var clipPointY = tRot.s * tVec.x + tRot.c * tVec.y + xfB.p.y;
                    var vv = (clipPointX - planePointX) * this.normal.x + (clipPointY - planePointY) * this.normal.y;
                    var cAX = clipPointX + (radiusA - vv) * this.normal.x;
                    var cAY = clipPointY + (radiusA - vv) * this.normal.y;
                    var cBX = clipPointX - radiusB * this.normal.x;
                    var cBY = clipPointY - radiusB * this.normal.y;
                    this.points[i].Set(0.5 * (cAX + cBX), 0.5 * (cAY + cBY));
                }
            }
            break;
        case b2Manifold.e_faceB:
            {
                tRot = xfB.q;
                tVec = manifold.localNormal;
                this.normal.x = tRot.c * tVec.x - tRot.s * tVec.y;
                this.normal.y = tRot.s * tVec.x + tRot.c * tVec.y;
                tVec = manifold.localPoint;
                var planePointX = tRot.c * tVec.x - tRot.s * tVec.y + xfB.p.x;
                var planePointY = tRot.s * tVec.x + tRot.c * tVec.y + xfB.p.y;
                tRot = xfA.q;
                for (var i = 0; i < manifold.pointCount; ++i) {
                    tVec = manifold.points[i].localPoint;
                    var clipPointX = tRot.c * tVec.x - tRot.s * tVec.y + xfA.p.x;
                    var clipPointY = tRot.s * tVec.x + tRot.c * tVec.y + xfA.p.y;
                    var vv = (clipPointX - planePointX) * this.normal.x + (clipPointY - planePointY) * this.normal.y;
                    var cBX = clipPointX + (radiusB - vv) * this.normal.x;
                    var cBY = clipPointY + (radiusB - vv) * this.normal.y;
                    var cAX = clipPointX - radiusA * this.normal.x;
                    var cAY = clipPointY - radiusA * this.normal.y;
                    this.points[i].Set(0.5 * (cAX + cBX), 0.5 * (cAY + cBY));
                }
                this.normal.x = -this.normal.x;
                this.normal.y = -this.normal.y;
            }
            break;
        }
    }
    b2ClipVertex.b2ClipVertex = function() {
        this.v = new b2Vec2();
        this.id = new b2ContactID();
    };
    b2ClipVertex.prototype.Copy = function() {
        var copy = new b2ClipVertex();
        copy.Set(this);
        return copy;
    }
    b2ClipVertex.prototype.Set = function(other) {
        this.v.SetV(other.v);
        this.id.Set(other.id);
    }
    b2ContactFeature.b2ContactFeature = function() {};
    Object.defineProperty(b2ContactFeature.prototype, 'indexA', {
        enumerable: false,
        configurable: true,
        get: function() {
            return this._indexA;
        }
    });
    Object.defineProperty(b2ContactFeature.prototype, 'indexA', {
        enumerable: false,
        configurable: true,
        set: function(value) {
            if (value === undefined) value = 0;
            this._indexA = value;
            this._m_id._key = (this._m_id._key & 0xffffff00) | (this._indexA & 0x000000ff);
        }
    });
    Object.defineProperty(b2ContactFeature.prototype, 'indexB', {
        enumerable: false,
        configurable: true,
        get: function() {
            return this._indexB;
        }
    });
    Object.defineProperty(b2ContactFeature.prototype, 'indexB', {
        enumerable: false,
        configurable: true,
        set: function(value) {
            if (value === undefined) value = 0;
            this._indexB = value;
            this._m_id._key = (this._m_id._key & 0xffff00ff) | ((this._indexB << 8) & 0x0000ff00);
        }
    });
    Object.defineProperty(b2ContactFeature.prototype, 'typeA', {
        enumerable: false,
        configurable: true,
        get: function() {
            return this._typeA;
        }
    });
    Object.defineProperty(b2ContactFeature.prototype, 'typeA', {
        enumerable: false,
        configurable: true,
        set: function(value) {
            if (value === undefined) value = 0;
            this._typeA = value;
            this._m_id._key = (this._m_id._key & 0xff00ffff) | ((this._typeA << 16) & 0x00ff0000);
        }
    });
    Object.defineProperty(b2ContactFeature.prototype, 'typeB', {
        enumerable: false,
        configurable: true,
        get: function() {
            return this._typeB;
        }
    });
    Object.defineProperty(b2ContactFeature.prototype, 'typeB', {
        enumerable: false,
        configurable: true,
        set: function(value) {
            if (value === undefined) value = 0;
            this._typeB = value;
            this._m_id._key = (this._m_id._key & 0x00ffffff) | ((this._typeB << 24) & 0xff000000);
        }
    });
    Box2D.postDefs.push(function() {
        Box2D.Collision.b2ContactFeature.e_vertex = 0;
        Box2D.Collision.b2ContactFeature.e_face = 1;
    });
})();
(function() {
    var b2Color = Box2D.Common.b2Color,
        b2Settings = Box2D.Common.b2Settings,
        b2Timer = Box2D.Common.b2Timer,
        b2CircleShape = Box2D.Collision.Shapes.b2CircleShape,
        b2EdgeShape = Box2D.Collision.Shapes.b2EdgeShape,
        b2ChainShape = Box2D.Collision.Shapes.b2ChainShape,
        b2MassData = Box2D.Collision.Shapes.b2MassData,
        b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape,
        b2Shape = Box2D.Collision.Shapes.b2Shape,
        b2Mat22 = Box2D.Common.Math.b2Mat22,
        b2Mat33 = Box2D.Common.Math.b2Mat33,
        b2Rot = Box2D.Common.Math.b2Rot,
        b2Math = Box2D.Common.Math.b2Math,
        b2Sweep = Box2D.Common.Math.b2Sweep,
        b2Transform = Box2D.Common.Math.b2Transform,
        b2Vec2 = Box2D.Common.Math.b2Vec2,
        b2Vec3 = Box2D.Common.Math.b2Vec3,
        b2Body = Box2D.Dynamics.b2Body,
        b2BodyDef = Box2D.Dynamics.b2BodyDef,
        b2ContactFilter = Box2D.Dynamics.b2ContactFilter,
        b2ContactImpulse = Box2D.Dynamics.b2ContactImpulse,
        b2ContactListener = Box2D.Dynamics.b2ContactListener,
        b2ContactManager = Box2D.Dynamics.b2ContactManager,
        b2Draw = Box2D.Dynamics.b2Draw,
        b2DestructionListener = Box2D.Dynamics.b2DestructionListener,
        b2Filter = Box2D.Dynamics.b2Filter,
        b2Fixture = Box2D.Dynamics.b2Fixture,
        b2FixtureDef = Box2D.Dynamics.b2FixtureDef,
        b2FixtureProxy = Box2D.Dynamics.b2FixtureProxy,
        b2Island = Box2D.Dynamics.b2Island,
        b2Position = Box2D.Dynamics.b2Position,
        b2Profile = Box2D.Dynamics.b2Profile,
        b2SolverData = Box2D.Dynamics.b2SolverData,
        b2TimeStep = Box2D.Dynamics.b2TimeStep,
        b2Velocity = Box2D.Dynamics.b2Velocity,
        b2World = Box2D.Dynamics.b2World,
        b2AABB = Box2D.Collision.b2AABB,
        b2Collision = Box2D.Collision.b2Collision,
        b2ContactID = Box2D.Collision.b2ContactID,
        b2Distance = Box2D.Collision.b2Distance,
        b2DistanceInput = Box2D.Collision.b2DistanceInput,
        b2DistanceOutput = Box2D.Collision.b2DistanceOutput,
        b2DistanceProxy = Box2D.Collision.b2DistanceProxy,
        b2DynamicTree = Box2D.Collision.b2DynamicTree,
        b2BroadPhase = Box2D.Collision.b2BroadPhase,
        b2TreeNode = Box2D.Collision.b2TreeNode,
        b2Pair = Box2D.Collision.b2Pair,
        b2Manifold = Box2D.Collision.b2Manifold,
        b2ManifoldPoint = Box2D.Collision.b2ManifoldPoint,
        b2RayCastInput = Box2D.Collision.b2RayCastInput,
        b2RayCastOutput = Box2D.Collision.b2RayCastOutput,
        b2SeparationFunction = Box2D.Collision.b2SeparationFunction,
        b2Simplex = Box2D.Collision.b2Simplex,
        b2SimplexCache = Box2D.Collision.b2SimplexCache,
        b2SimplexVertex = Box2D.Collision.b2SimplexVertex,
        b2TOIInput = Box2D.Collision.b2TOIInput,
        b2TOIOutput = Box2D.Collision.b2TOIOutput,
        b2WorldManifold = Box2D.Collision.b2WorldManifold,
        b2ClipVertex = Box2D.Collision.b2ClipVertex,
        b2ContactFeature = Box2D.Collision.b2ContactFeature,
        IBroadPhase = Box2D.Collision.IBroadPhase,
        b2Rope = Box2D.Rope.b2Rope,
        b2RopeDef = Box2D.Rope.b2RopeDef;

    Box2D.inherit(b2CircleShape, Box2D.Collision.Shapes.b2Shape);
    b2CircleShape.prototype.__super = Box2D.Collision.Shapes.b2Shape.prototype;
    b2CircleShape.b2CircleShape = function() {
        Box2D.Collision.Shapes.b2Shape.b2Shape.apply(this, arguments);
        this.m_p = new b2Vec2();
    };
    b2CircleShape.prototype.Copy = function() {
        var s = new b2CircleShape();
        s.SetByOther(this);
        return s;
    }
    b2CircleShape.prototype.SetByOther = function(other) {
        this.__super.SetByOther.call(this, other);
        if (Box2D.is(other, b2CircleShape)) {
            this.m_p.SetV(other.m_p);
        }
    }
    b2CircleShape.prototype.TestPoint = function(transform, p) {
        var tRot = transform.q;
        var cX = tRot.c * this.m_p.x - tRot.s * this.m_p.y;
        var cY = tRot.s * this.m_p.x + tRot.c * this.m_p.y;
        cX += transform.p.x;
        cY += transform.p.y;
        var dX = p.x - cX;
        var dY = p.y - cY;
        return (dX * dX + dY * dY) <= this.m_radius * this.m_radius;
    }
    b2CircleShape.prototype.RayCast = function(output, input, transform, childIndex) {
        var tRot = transform.q;
        var positionX = transform.p.x + (tRot.c * this.m_p.x - tRot.s * this.m_p.y);
        var positionY = transform.p.y + (tRot.s * this.m_p.x + tRot.c * this.m_p.y);
        var sX = input.p1.x - positionX;
        var sY = input.p1.y - positionY;
        var b = (sX * sX + sY * sY) - this.m_radius * this.m_radius;
        var rX = input.p2.x - input.p1.x;
        var rY = input.p2.y - input.p1.y;
        var c = (sX * rX + sY * rY);
        var rr = (rX * rX + rY * rY);
        var sigma = c * c - rr * b;
        if (sigma < 0.0 || rr < Number.MIN_VALUE) {
            return false;
        }
        var a = (-(c + Math.sqrt(sigma)));
        if (0.0 <= a && a <= input.maxFraction * rr) {
            a /= rr;
            output.fraction = a;
            output.normal.x = sX + a * rX;
            output.normal.y = sY + a * rY;
            output.normal.Normalize();
            return true;
        }
        return false;
    }
    b2CircleShape.prototype.ComputeAABB = function(aabb, transform, childIndex) {
        var tRot = transform.q;
        var pX = transform.p.x + (tRot.c * this.m_p.x - tRot.s * this.m_p.y);
        var pY = transform.p.y + (tRot.s * this.m_p.x + tRot.c * this.m_p.y);
        aabb.lowerBound.Set(pX - this.m_radius, pY - this.m_radius);
        aabb.upperBound.Set(pX + this.m_radius, pY + this.m_radius);
    }
    b2CircleShape.prototype.ComputeMass = function(massData, density) {
        massData.mass = density * b2Settings.b2_pi * this.m_radius * this.m_radius;
        massData.center.SetV(this.m_p);
        massData.I = massData.mass * (0.5 * this.m_radius * this.m_radius + (this.m_p.x * this.m_p.x + this.m_p.y * this.m_p.y));
    }
/*
    b2CircleShape.prototype.ComputeSubmergedArea = function(normal, offset, xf, c) {
        if (offset === undefined) offset = 0;
        var p = b2Math.MulXV(xf, this.m_p);
        var l = (-(b2Math.DotVV(normal, p) - offset));
        if (l < (-this.m_radius) + Number.MIN_VALUE) {
            return 0;
        }
        if (l > this.m_radius) {
            c.SetV(p);
            return Math.PI * this.m_radius * this.m_radius;
        }
        var r2 = this.m_radius * this.m_radius;
        var l2 = l * l;
        var area = r2 * (Math.asin(l / this.m_radius) + Math.PI / 2) + l * Math.sqrt(r2 - l2);
        var com = (-2 / 3 * Math.pow(r2 - l2, 1.5) / area);
        c.x = p.x + normal.x * com;
        c.y = p.y + normal.y * com;
        return area;
    }
    */
    b2CircleShape.prototype.b2CircleShape = function(radius) {
        this.__super.b2Shape.call(this);
        this.m_type = b2Shape.e_circle;
        this.m_radius = radius;
        this.m_p.SetZero();
    }
    b2CircleShape.prototype.GetChildCount = function() {
        return 1;
    }
    b2CircleShape.prototype.GetSupport = function(d) {
        return 0;
    }
    b2CircleShape.prototype.GetSupportVertex = function(d) {
        return this.m_p;
    }
    b2CircleShape.prototype.GetVertex = function(index) {
        return this.m_p;
    }
    b2CircleShape.prototype.GetVertexCount = function() {
        return 1;
    }
    Box2D.inherit(b2EdgeShape, Box2D.Collision.Shapes.b2Shape);
    b2EdgeShape.prototype.__super = Box2D.Collision.Shapes.b2Shape.prototype;
    b2EdgeShape.b2EdgeShape = function() {
        Box2D.Collision.Shapes.b2Shape.b2Shape.apply(this, arguments);
        this.m_vertex1 = new b2Vec2();
        this.m_vertex2 = new b2Vec2();
        this.m_vertex0 = new b2Vec2();
        this.m_vertex3 = new b2Vec2();
        this.m_hasVertex0 = false;
        this.m_hasVertex3 = false;
    };
    b2EdgeShape.prototype.Copy = function() {
        var s = new b2EdgeShape();
        s.SetByOther(this);
        return s;
    }
    b2EdgeShape.prototype.SetByOther = function(other) {
        this.__super.SetByOther.call(this, other);
        if (Box2D.is(other, b2EdgeShape)) {
            this.m_vertex1.SetV(other.m_vertex1);
            this.m_vertex2.SetV(other.m_vertex2);
            this.m_vertex0.SetV(other.m_vertex0);
            this.m_vertex3.SetV(other.m_vertex3);
            this.m_hasVertex0 = other.m_hasVertex0;
            this.m_hasVertex3 = other.m_hasVertex3;
        }
    }
    b2EdgeShape.prototype.TestPoint = function(transform, p) {
        return false;
    }
    b2EdgeShape.prototype.RayCast = function(output, input, transform) {
        var tRot;
        var rX = input.p2.x - input.p1.x;
        var rY = input.p2.y - input.p1.y;
        tRot = transform.q;
        var v1X = transform.p.x + (tRot.c * this.m_vertex1.x - tRot.s * this.m_vertex1.y);
        var v1Y = transform.p.y + (tRot.s * this.m_vertex1.x + tRot.c * this.m_vertex1.y);
        var nX = transform.p.y + (tRot.s * this.m_vertex2.x + tRot.c * this.m_vertex2.y) - v1Y;
        var nY = (-(transform.p.x + (tRot.c * this.m_vertex2.x - tRot.s * this.m_vertex2.y) - v1X));
        var k_slop = 100.0 * Number.MIN_VALUE;
        var denom = (-(rX * nX + rY * nY));
        if (denom > k_slop) {
            var bX = input.p1.x - v1X;
            var bY = input.p1.y - v1Y;
            var a = (bX * nX + bY * nY);
            if (0.0 <= a && a <= input.maxFraction * denom) {
                var mu2 = (-rX * bY) + rY * bX;
                if ((-k_slop * denom) <= mu2 && mu2 <= denom * (1.0 + k_slop)) {
                    a /= denom;
                    output.fraction = a;
                    var nLen = Math.sqrt(nX * nX + nY * nY);
                    output.normal.x = nX / nLen;
                    output.normal.y = nY / nLen;
                    return true;
                }
            }
        }
        return false;
    }
    b2EdgeShape.prototype.ComputeAABB = function(aabb, xf, childIndex) {
        var v1 = b2Math.MulXV(xf, this.m_vertex1);
        var v2 = b2Math.MulXV(xf, this.m_vertex2);
        var lower = b2Math.MinV(v1, v2);
        var upper = b2Math.MaxV(v1, v2);
        var r = new b2Vec2(this.m_radius, this.m_radius);
        lower.Subtract(r);
        upper.Add(r);
        aabb.lowerBound = lower;
        aabb.upperBound = upper;
    }
    b2EdgeShape.prototype.ComputeMass = function(massData, density) {
        massData.mass = 0;
        var cx = (this.m_vertex1.x + this.m_vertex2.x) / 2.0;
        var cy = (this.m_vertex1.y + this.m_vertex2.y) / 2.0;
        massData.center.Set(cx, cy);
        massData.I = 0;
    }
    b2EdgeShape.prototype.GetChildCount = function() {
        return 1;
    }
    b2EdgeShape.prototype.Set = function(v1, v2) {
        this.m_vertex1.SetV(v1);
        this.m_vertex2.SetV(v2);
        this.m_hasVertex0 = false;
        this.m_hasVertex3 = false;
    }
    b2EdgeShape.prototype.b2EdgeShape = function(v1, v2) {
        this.__super.b2Shape.call(this);
        this.m_type = b2Shape.e_edge;
        this.m_radius = b2Settings.b2_polygonRadius;
        if (v1 != undefined && v2 != undefined)
            this.Set(v1, v2);
    }
    Box2D.inherit(b2ChainShape, Box2D.Collision.Shapes.b2Shape);
    b2ChainShape.prototype.__super = Box2D.Collision.Shapes.b2Shape.prototype;
    b2ChainShape.b2ChainShape = function() {
        Box2D.Collision.Shapes.b2Shape.b2Shape.apply(this, arguments);
        this.m_vertices = null;
        this.m_prevVertex = new b2Vec2();
        this.m_nextVertex = new b2Vec2();
        this.m_hasPrevVertex = false;
        this.m_hasNextVertex = false;
    };
    b2ChainShape.prototype.Copy = function() {
        var s = new b2ChainShape();
        s.SetByOther(this);
        return s;
    }
    b2ChainShape.prototype.SetByOther = function(other) {
        this.__super.SetByOther.call(this, other);
        if (Box2D.is(other, b2ChainShape)) {
            this.m_count = other.m_count;
            this.m_vertices = new Vector(other.m_vertices.length);
            for (var i = 0; i < other.m_vertices.length; i++) {
                this.m_vertices[i] = new b2Vec2();
                this.m_vertices[i].SetV(other.m_vertices[i]);
            }
            this.m_prevVertex.SetV(other.m_prevVertex);
            this.m_nextVertex.SetV(other.m_nextVertex);
            this.m_hasPrevVertex = other.m_hasPrevVertex;
            this.m_hasNextVertex = other.m_hasNextVertex;
        }
    }
    b2ChainShape.prototype.CreateLoop = function(vertices) {
        b2Settings.b2Assert(this.m_count === 0 && this.m_vertices === null);
        b2Settings.b2Assert(vertices.length >= 3);
        this.m_count = vertices.length + 1;
        this.m_vertices = new Vector(this.m_count);
        for (var i = 0; i < vertices.length; i++) {
            this.m_vertices[i] = vertices[i].Copy();
        }
        this.m_vertices[vertices.length] = this.m_vertices[0].Copy();
        this.m_prevVertex.SetV(this.m_vertices[this.m_count - 2]);
        this.m_nextVertex.SetV(this.m_vertices[1]);
        this.m_hasPrevVertex = true;
        this.m_hasNextVertex = true;
    }
    b2ChainShape.prototype.CreateChain = function(vertices) {
        b2Settings.b2Assert(this.m_count === 0 && this.m_vertices === null);
        b2Settings.b2Assert(vertices.length >= 2);
        this.m_count = vertices.length;
        this.m_vertices = new Vector(this.m_count);
        for (var i = 0; i < vertices.length; i++) {
            this.m_vertices[i] = vertices[i].Copy();
        }
        this.m_hasPrevVertex = false;
        this.m_hasNextVertex = false;
    }
    b2ChainShape.prototype.SetPrevVertex = function(prevVertex) {
        this.m_prevVertex.SetV(prevVertex);
        this.m_hasPrevVertex = true;
    }
    b2ChainShape.prototype.SetNextVertex = function(nextVertex) {
        this.m_nextVertex.SetV(nextVertex);
        this.m_hasNextVertex = true;
    }
    b2ChainShape.prototype.GetChildEdge = function(edge, index) {
        b2Settings.b2Assert(0 <= index && index < this.m_count - 1);
        edge.m_type = b2Shape.e_edge;
        edge.m_radius = this.m_radius;
        edge.m_vertex1 = this.m_vertices[index];
        edge.m_vertex2 = this.m_vertices[index + 1];

        if (index > 0) {
            edge.m_vertex0 = this.m_vertices[index - 1];
            edge.m_hasVertex0 = true;
        } else {
            edge.m_vertex0 = this.m_prevVertex;
            edge.m_hasVertex0 = this.m_hasPrevVertex;
        }
        if (index < this.m_count - 2) {
            edge.m_vertex3 = this.m_vertices[index + 2];
            edge.m_hasVertex3 = true;
        } else {
            edge.m_vertex3 = this.m_nextVertex;
            edge.m_hasVertex3 = this.m_hasNextVertex;
        }
    }
    b2ChainShape.prototype.TestPoint = function(transform, p) {
        return false;
    }
    b2ChainShape.prototype.RayCast = function(output, input, xf, childIndex) {
        b2Settings.b2Assert(childIndex < this.m_count);

        var edgeShape = new b2EdgeShape();

        var i1 = childIndex;
        var i2 = childIndex + 1;
        if (i2 == this.m_count) {
            i2 = 0;
        }

        edgeShape.m_vertex1 = m_vertices[i1].Copy();
        edgeShape.m_vertex2 = m_vertices[i2].Copy();

        return edgeShape.RayCast(output, input, xf, 0);
    }
    b2ChainShape.prototype.ComputeAABB = function(aabb, xf, index) {
        b2Settings.b2Assert(0 <= index && index < this.m_count);

        var i1 = index;
        var i2 = index + 1;
        if (i2 == this.m_count) {
            i2 = 0;
        }
        var v1 = b2Math.MulXV(xf, this.m_vertices[i1]);
        var v2 = b2Math.MulXV(xf, this.m_vertices[i2]);
        aabb.lowerBound = b2Math.MinV(v1, v2);
        aabb.upperBound = b2Math.MaxV(v1, v2);
    }
    b2ChainShape.prototype.ComputeMass = function(massData, density) {
        if (density === undefined) density = 0;
        massData.mass = 0;
        massData.center.SetZero();
        massData.I = 0;
    }
    b2ChainShape.prototype.GetChildCount = function() {
        return this.m_count - 1;
    }
    b2ChainShape.prototype.b2ChainShape = function() {
        this.__super.b2Shape.call(this);
        this.m_type = b2Shape.e_chain;
        this.m_radius = b2Settings.b2_polygonRadius;
        this.m_vertices = null;
        this.m_count = 0;
        this.m_hasPrevVertex = false;
        this.m_hasNextVertex = false;
    }
    b2MassData.b2MassData = function() {
        this.mass = 0.0;
        this.center = new b2Vec2(0, 0);
        this.I = 0.0;
    };
    b2MassData.prototype.Set = function(other) {
        this.mass = other.mass;
        this.center.SetV(other.center);
        this.I = other.I;
    };
    b2MassData.prototype.Copy = function() {
        var copy = new b2MassData();
        copy.Set(this);
        return copy;
    }
    Box2D.inherit(b2PolygonShape, Box2D.Collision.Shapes.b2Shape);
    b2PolygonShape.prototype.__super = Box2D.Collision.Shapes.b2Shape.prototype;
    b2PolygonShape.b2PolygonShape = function() {
        Box2D.Collision.Shapes.b2Shape.b2Shape.apply(this, arguments);
    };
    b2PolygonShape.prototype.Copy = function() {
        var s = new b2PolygonShape();
        s.SetByOther(this);
        return s;
    }
    b2PolygonShape.prototype.GetChildCount = function() {
        return 1;
    }
    b2PolygonShape.prototype.SetByOther = function(other) {
        this.__super.SetByOther.call(this, other);
        if (Box2D.is(other, b2PolygonShape)) {
            this.m_centroid.SetV(other.m_centroid);
            this.m_vertexCount = other.m_vertexCount;
            this.Reserve(this.m_vertexCount);
            for (var i = 0; i < this.m_vertexCount; i++) {
                this.m_vertices[i].SetV(other.m_vertices[i]);
                this.m_normals[i].SetV(other.m_normals[i]);
            }
        }
    }
    b2PolygonShape.prototype.Set = function(vertices) {
        this.m_vertexCount = vertices.length;
        this.Reserve(this.m_vertexCount);        
        for (var i = 0; i < vertices.length; ++i) {
            this.m_vertices[i].SetV(vertices[i]);
        }
        for (var i = 0; i < this.m_vertexCount; ++i) {
            var i1 = i;
            var i2 = i + 1 < this.m_vertexCount ? i + 1 : 0;
            var edge = b2Math.SubtractVV(this.m_vertices[i2], this.m_vertices[i1]);
            b2Settings.b2Assert(edge.LengthSquared() > b2Settings.b2_epsilon * b2Settings.b2_epsilon);
            this.m_normals[i] = b2Math.CrossVF(edge, 1.0);
            this.m_normals[i].Normalize();
        }
        this.m_centroid = b2PolygonShape.ComputeCentroid(this.m_vertices, this.m_vertexCount);
    }
    b2PolygonShape.prototype.SetAsVector = function(vertices, vertexCount) {
        if (vertexCount === undefined) vertexCount = 0;
        if (vertexCount == 0) vertexCount = vertices.length;
        b2Settings.b2Assert(2 <= vertexCount);
        this.m_vertexCount = vertexCount;
        this.Reserve(vertexCount);
        var i = 0;
        for (i = 0;
        i < this.m_vertexCount; i++) {
            this.m_vertices[i].SetV(vertices[i]);
        }
        for (i = 0;
        i < this.m_vertexCount; ++i) {
            var i1 = parseInt(i);
            var i2 = parseInt(i + 1 < this.m_vertexCount ? i + 1 : 0);
            var edge = b2Math.SubtractVV(this.m_vertices[i2], this.m_vertices[i1]);
            b2Settings.b2Assert(edge.LengthSquared() > Number.MIN_VALUE);
            this.m_normals[i].SetV(b2Math.CrossVF(edge, 1.0));
            this.m_normals[i].Normalize();
        }
        this.m_centroid = b2PolygonShape.ComputeCentroid(this.m_vertices, this.m_vertexCount);
    }
    b2PolygonShape.prototype.SetAsBox = function(hx, hy, center, angle) {
        if (center != undefined && angle != undefined) {
            this.SetAsOrientedBox(hx, hy, center, angle);
            return;
        }
        this.m_vertexCount = 4;
        this.Reserve(4);
        this.m_vertices[0].Set((-hx), (-hy));
        this.m_vertices[1].Set(hx, (-hy));
        this.m_vertices[2].Set(hx, hy);
        this.m_vertices[3].Set((-hx), hy);
        this.m_normals[0].Set(0.0, (-1.0));
        this.m_normals[1].Set(1.0, 0.0);
        this.m_normals[2].Set(0.0, 1.0);
        this.m_normals[3].Set((-1.0), 0.0);
        this.m_centroid.SetZero();
    }
    b2PolygonShape.prototype.SetAsOrientedBox = function(hx, hy, center, angle) {
        this.m_vertexCount = 4;
        this.Reserve(4);
        this.m_vertices[0].Set((-hx), (-hy));
        this.m_vertices[1].Set(hx, (-hy));
        this.m_vertices[2].Set(hx, hy);
        this.m_vertices[3].Set((-hx), hy);
        this.m_normals[0].Set(0.0, (-1.0));
        this.m_normals[1].Set(1.0, 0.0);
        this.m_normals[2].Set(0.0, 1.0);
        this.m_normals[3].Set((-1.0), 0.0);
        this.m_centroid.SetV(center);
        var xf = new b2Transform();
        xf.Set(center, angle);
        for (var i = 0; i < this.m_vertexCount; ++i) {
            this.m_vertices[i] = b2Math.MulXV(xf, this.m_vertices[i]);
            this.m_normals[i] = b2Math.MulRV(xf.q, this.m_normals[i]);
        }
    }
    b2PolygonShape.prototype.TestPoint = function(xf, p) {
        var tVec;
        var tRot = xf.q;
        var tX = p.x - xf.p.x;
        var tY = p.y - xf.p.y;
        var pLocalX = (tX * tRot.c + tY * tRot.s);
        var pLocalY = (-tX * tRot.s + tY * tRot.c);
        for (var i = 0; i < this.m_vertexCount; ++i) {
            tVec = this.m_vertices[i];
            tX = pLocalX - tVec.x;
            tY = pLocalY - tVec.y;
            tVec = this.m_normals[i];
            var dot = (tVec.x * tX + tVec.y * tY);
            if (dot > 0.0) {
                return false;
            }
        }
        return true;
    }
    b2PolygonShape.prototype.RayCast = function(output, input, transform, childIndex) {
        var lower = 0.0;
        var upper = input.maxFraction;
        var tX = 0;
        var tY = 0;
        var tRot;
        var tVec;
        tX = input.p1.x - transform.p.x;
        tY = input.p1.y - transform.p.y;
        tRot = transform.q;
        var p1X = (tX * tRot.c + tY * tRot.s);
        var p1Y = (-tX * tRot.s + tY * tRot.c);
        tX = input.p2.x - transform.p.x;
        tY = input.p2.y - transform.p.y;
        tRot = transform.q;
        var p2X = (tX * tRot.c + tY * tRot.s);
        var p2Y = (-tX * tRot.s + tY * tRot.c);
        var dX = p2X - p1X;
        var dY = p2Y - p1Y;
        var index = parseInt((-1));
        for (var i = 0; i < this.m_vertexCount; ++i) {
            tVec = this.m_vertices[i];
            tX = tVec.x - p1X;
            tY = tVec.y - p1Y;
            tVec = this.m_normals[i];
            var numerator = (tVec.x * tX + tVec.y * tY);
            var denominator = (tVec.x * dX + tVec.y * dY);
            if (denominator == 0.0) {
                if (numerator < 0.0) {
                    return false;
                }
            } else {
                if (denominator < 0.0 && numerator < lower * denominator) {
                    lower = numerator / denominator;
                    index = i;
                } else if (denominator > 0.0 && numerator < upper * denominator) {
                    upper = numerator / denominator;
                }
            }
            if (upper < lower - Number.MIN_VALUE) {
                return false;
            }
        }
        if (index >= 0) {
            output.fraction = lower;
            tRot = transform.q;
            tVec = this.m_normals[index];
            output.normal.x = (tRot.c * tVec.x - tRot.s * tVec.y);
            output.normal.y = (tRot.s * tVec.x + tRot.c * tVec.y);
            return true;
        }
        return false;
    }
    b2PolygonShape.prototype.ComputeAABB = function(aabb, xf, childIndex) {
        var tRot = xf.q;
        var tVec = this.m_vertices[0];
        var lowerX = (tRot.c * tVec.x - tRot.s * tVec.y) + xf.p.x;
        var lowerY = (tRot.s * tVec.x + tRot.c * tVec.y) + xf.p.y;
        var upperX = lowerX;
        var upperY = lowerY;
        for (var i = 1; i < this.m_vertexCount; ++i) {
            tVec = this.m_vertices[i];
            var vX = (tRot.c * tVec.x - tRot.s * tVec.y) + xf.p.x;
            var vY = (tRot.s * tVec.x + tRot.c * tVec.y) + xf.p.y;
            lowerX = lowerX < vX ? lowerX : vX;
            lowerY = lowerY < vY ? lowerY : vY;
            upperX = upperX > vX ? upperX : vX;
            upperY = upperY > vY ? upperY : vY;
        }
        aabb.lowerBound.x = lowerX - this.m_radius;
        aabb.lowerBound.y = lowerY - this.m_radius;
        aabb.upperBound.x = upperX + this.m_radius;
        aabb.upperBound.y = upperY + this.m_radius;
    }
    b2PolygonShape.prototype.ComputeMass = function(massData, density) {
        b2Settings.b2Assert(this.m_vertexCount >= 3);

        var centerX = 0.0;
        var centerY = 0.0;
        var area = 0.0;
        var I = 0.0;
        var p1X = 0.0;
        var p1Y = 0.0;
        for (var i = 0; i < this.m_vertexCount; ++i) {
            p1X += this.m_vertices[i].x;
            p1Y += this.m_vertices[i].y;
        }
        p1X *= 1.0 / this.m_vertexCount;
        p1Y *= 1.0 / this.m_vertexCount;

        var k_inv3 = 1.0 / 3.0;
        for (var i = 0; i < this.m_vertexCount; ++i) {
            var e1X = this.m_vertices[i].x - p1X;
            var e1Y = this.m_vertices[i].y - p1Y;
            var e2 = i + 1 < this.m_vertexCount ? this.m_vertices[parseInt(i + 1)] : this.m_vertices[0];
            var e2X = e2.x - p1X;
            var e2Y = e2.y - p1Y;
            var D = e1X * e2Y - e1Y * e2X;
            var triangleArea = 0.5 * D;
            area += triangleArea;
            centerX += triangleArea * k_inv3 * (e1X + e2X);
            centerY += triangleArea * k_inv3 * (e1Y + e2Y);
            var intx2 = e1X * e1X + e2X * e1X + e2X * e2X;
            var inty2 = e1Y * e1Y + e2Y * e1Y + e2Y * e2Y;
            I += 0.25 * k_inv3 * D * (intx2 + inty2);
        }
        massData.mass = density * area;
        centerX *= 1.0 / area;
        centerY *= 1.0 / area;
        massData.center.Set(centerX + p1X, centerY + p1Y);
        massData.I = density * I;
        massData.I += massData.mass * (b2Math.DotVV(massData.center, massData.center) - (centerX * centerX + centerY * centerY));
    }
    b2PolygonShape.prototype.GetVertexCount = function() {
        return this.m_vertexCount;
    }
    b2PolygonShape.prototype.GetVertex = function(index) {
        b2Settings.b2Assert(0 <= index && index < this.m_vertexCount);
        return this.m_vertices[index];
    }
    b2PolygonShape.prototype.b2PolygonShape = function() {
        this.__super.b2Shape.call(this);
        this.m_type = b2Shape.e_polygon;
        this.m_radius = b2Settings.b2_polygonRadius;
        this.m_centroid = new b2Vec2();
        this.m_vertices = new Vector();
        this.m_normals = new Vector();
    }
    b2PolygonShape.prototype.Reserve = function(count) {
        for (var i = parseInt(this.m_vertices.length); i < count; i++) {
            this.m_vertices[i] = new b2Vec2();
            this.m_normals[i] = new b2Vec2();
        }
    }
    b2PolygonShape.ComputeCentroid = function(vs, count) {
        b2Settings.b2Assert(count >= 3);
        var c = new b2Vec2();
        var area = 0.0;
        var p1X = 0.0;
        var p1Y = 0.0;
        var inv3 = 1.0 / 3.0;
        for (var i = 0; i < count; ++i) {
            var p2 = vs[i];
            var p3 = i + 1 < count ? vs[parseInt(i + 1)] : vs[0];
            var e1X = p2.x - p1X;
            var e1Y = p2.y - p1Y;
            var e2X = p3.x - p1X;
            var e2Y = p3.y - p1Y;
            var D = (e1X * e2Y - e1Y * e2X);
            var triangleArea = 0.5 * D;
            area += triangleArea;
            c.x += triangleArea * inv3 * (p1X + p2.x + p3.x);
            c.y += triangleArea * inv3 * (p1Y + p2.y + p3.y);
        }
        c.x *= 1.0 / area;
        c.y *= 1.0 / area;
        return c;
    }
    b2Shape.b2Shape = function() {};
    b2Shape.prototype.Copy = function() {
        return null;
    }
    b2Shape.prototype.SetByOther = function(other) {
        this.m_type = other.m_type;
        this.m_radius = other.m_radius;
    }
    b2Shape.prototype.GetType = function() {
        return this.m_type;
    }
    b2Shape.prototype.GetChildCount = function() {
        return 0;
    }
    b2Shape.prototype.TestPoint = function(xf, p) {
        return false;
    }
    b2Shape.prototype.RayCast = function(output, input, transform) {
        return false;
    }
    b2Shape.prototype.ComputeAABB = function(aabb, xf) {}
    b2Shape.prototype.ComputeMass = function(massData, density) {}
    b2Shape.prototype.b2Shape = function() {
        this.m_type = b2Shape.e_unknownShape;
        this.m_radius = b2Settings.b2_linearSlop;
    }
    Box2D.postDefs.push(function() {
        Box2D.Collision.Shapes.b2Shape.e_unknownShape = parseInt((-1));
        Box2D.Collision.Shapes.b2Shape.e_circle = 0;
        Box2D.Collision.Shapes.b2Shape.e_edge = 1;
        Box2D.Collision.Shapes.b2Shape.e_polygon = 2;
        Box2D.Collision.Shapes.b2Shape.e_chain = 3;
        Box2D.Collision.Shapes.b2Shape.e_typeCount = 4;
    });
})();
(function() {
    var b2Color = Box2D.Common.b2Color,
        b2Settings = Box2D.Common.b2Settings,
        b2Timer = Box2D.Common.b2Timer,
        b2Mat22 = Box2D.Common.Math.b2Mat22,
        b2Mat33 = Box2D.Common.Math.b2Mat33,
        b2Rot = Box2D.Common.Math.b2Rot,
        b2Math = Box2D.Common.Math.b2Math,
        b2Sweep = Box2D.Common.Math.b2Sweep,
        b2Transform = Box2D.Common.Math.b2Transform,
        b2Vec2 = Box2D.Common.Math.b2Vec2,
        b2Vec3 = Box2D.Common.Math.b2Vec3;

    b2Color.b2Color = function() {
        this.r = 0;
        this.g = 0;
        this.b = 0;
    };
    b2Color.prototype.b2Color = function(rr, gg, bb) {
        if (rr === undefined) rr = 0;
        if (gg === undefined) gg = 0;
        if (bb === undefined) bb = 0;
        this.r = b2Math.Clamp(rr, 0.0, 1.0);
        this.g = b2Math.Clamp(gg, 0.0, 1.0);
        this.b = b2Math.Clamp(bb, 0.0, 1.0);
    }
    b2Color.prototype.Set = function(rr, gg, bb) {
        this.r = b2Math.Clamp(rr, 0.0, 1.0);
        this.g = b2Math.Clamp(gg, 0.0, 1.0);
        this.b = b2Math.Clamp(bb, 0.0, 1.0);
    }
/*
   Object.defineProperty(b2Color.prototype, 'r', {
      enumerable: false,
      configurable: true,
      set: function (rr) {
         if (rr === undefined) rr = 0;
         this._r = Box2D.parseUInt(255 * b2Math.Clamp(rr, 0.0, 1.0));
      }
   });
   Object.defineProperty(b2Color.prototype, 'g', {
      enumerable: false,
      configurable: true,
      set: function (gg) {
         if (gg === undefined) gg = 0;
         this._g = Box2D.parseUInt(255 * b2Math.Clamp(gg, 0.0, 1.0));
      }
   });
   Object.defineProperty(b2Color.prototype, 'b', {
      enumerable: false,
      configurable: true,
      set: function (bb) {
         if (bb === undefined) bb = 0;
         this._b = Box2D.parseUInt(255 * b2Math.Clamp(bb, 0.0, 1.0));
      }
   });
   */
    Object.defineProperty(b2Color.prototype, 'color', {
        enumerable: false,
        configurable: true,
        get: function() {
            return (Box2D.parseUInt(this.r * 255) << 16) | (Box2D.parseUInt(this.g * 255) << 8) | (Box2D.parseUInt(this.b * 255));
        }
    });
    b2Settings.b2Settings = function() {};
    b2Settings.b2MixFriction = function(friction1, friction2) {
        return Math.sqrt(friction1 * friction2);
    }
    b2Settings.b2MixRestitution = function(restitution1, restitution2) {
        return restitution1 > restitution2 ? restitution1 : restitution2;
    }
    b2Settings.b2Assert = function(a) {
        if (!a) {
            throw "Assertion Failed";
        }
    }
    Box2D.postDefs.push(function() {
        Box2D.Common.b2Settings.VERSION = "2.2.1";
        Box2D.Common.b2Settings.USHRT_MAX = 0x0000ffff;
        Box2D.Common.b2Settings.b2_maxFloat = Number.MAX_VALUE;
        Box2D.Common.b2Settings.b2_epsilon = Number.MIN_VALUE;
        Box2D.Common.b2Settings.b2_pi = Math.PI;
        Box2D.Common.b2Settings.b2_maxManifoldPoints = 2;
        Box2D.Common.b2Settings.b2_maxPolygonVertices = 8;
        Box2D.Common.b2Settings.b2_aabbExtension = 0.1;
        Box2D.Common.b2Settings.b2_aabbMultiplier = 2.0;
        Box2D.Common.b2Settings.b2_linearSlop = 0.005;
        Box2D.Common.b2Settings.b2_angularSlop = 2.0 / 180.0 * b2Settings.b2_pi;
        Box2D.Common.b2Settings.b2_polygonRadius = 2.0 * b2Settings.b2_linearSlop;
        Box2D.Common.b2Settings.b2_maxSubSteps = 8;

        Box2D.Common.b2Settings.b2_maxTOIContacts = 32;
        Box2D.Common.b2Settings.b2_velocityThreshold = 1.0;
        Box2D.Common.b2Settings.b2_maxLinearCorrection = 0.2;
        Box2D.Common.b2Settings.b2_maxAngularCorrection = 8.0 / 180.0 * b2Settings.b2_pi;
        Box2D.Common.b2Settings.b2_maxTranslation = 2.0;
        Box2D.Common.b2Settings.b2_maxTranslationSquared = b2Settings.b2_maxTranslation * b2Settings.b2_maxTranslation;
        Box2D.Common.b2Settings.b2_maxRotation = 0.5 * b2Settings.b2_pi;
        Box2D.Common.b2Settings.b2_maxRotationSquared = b2Settings.b2_maxRotation * b2Settings.b2_maxRotation;
        Box2D.Common.b2Settings.b2_baumgarte = 0.2;
        Box2D.Common.b2Settings.b2_toiBaugarte = 0.75;

        Box2D.Common.b2Settings.b2_timeToSleep = 0.5;
        Box2D.Common.b2Settings.b2_linearSleepTolerance = 0.01;
        Box2D.Common.b2Settings.b2_angularSleepTolerance = 2.0 / 180.0 * b2Settings.b2_pi;
    });
    b2Timer.b2Timer = function() {};
    b2Timer.prototype.b2Timer = function() {
        this.Reset();
    }
    b2Timer.prototype.Reset = function() {
        var d = new Date();
        this.m_start_msec = d.getTime();
    }
    b2Timer.prototype.GetMilliseconds = function() {
        var d = new Date();
        var t = d.getTime();
        return t - this.m_start_msec;
    }

})();
(function() {
    var b2AABB = Box2D.Collision.b2AABB,
        b2Color = Box2D.Common.b2Color,
        b2Settings = Box2D.Common.b2Settings,
        b2Timer = Box2D.Common.b2Timer,
        b2Mat22 = Box2D.Common.Math.b2Mat22,
        b2Mat33 = Box2D.Common.Math.b2Mat33,
        b2Rot = Box2D.Common.Math.b2Rot,
        b2Math = Box2D.Common.Math.b2Math,
        b2Sweep = Box2D.Common.Math.b2Sweep,
        b2Transform = Box2D.Common.Math.b2Transform,
        b2Vec2 = Box2D.Common.Math.b2Vec2,
        b2Vec3 = Box2D.Common.Math.b2Vec3;

    b2Mat22.b2Mat22 = function() {
        this.ex = new b2Vec2();
        this.ey = new b2Vec2();
    };
    b2Mat22.prototype.b2Mat22 = function(p1, p2, p3, p4) {
        if (typeof(p1) === "number") {
            this.ex.x = p1;
            this.ex.y = p3;
            this.ey.x = p2;
            this.ey.y = p4;
        } else if (p1 instanceof b2Vec2) {
            this.Set(p1, p2);
        } else {
            //         this.SetIdentity();
        }
    }
    b2Mat22.prototype.Set = function(c1, c2) {
        this.ex.SetV(c1);
        this.ey.SetV(c2);
    }
    b2Mat22.prototype.Copy = function () {
        var mat = new b2Mat22();
        mat.SetM(this);
        return mat;
    }
    b2Mat22.prototype.SetM = function (m) {
        this.ex.SetV(m.ex);
        this.ey.SetV(m.ey);
    }
/*
   b2Mat22.prototype.AddM = function (m) {
      this.ex.x += m.ex.x;
      this.ex.y += m.ex.y;
      this.ey.x += m.ey.x;
      this.ey.y += m.ey.y;
   }
*/
    b2Mat22.prototype.SetIdentity = function() {
        this.ex.x = 1.0;
        this.ey.x = 0.0;
        this.ex.y = 0.0;
        this.ey.y = 1.0;
    }
    b2Mat22.prototype.SetZero = function() {
        this.ex.x = 0.0;
        this.ey.x = 0.0;
        this.ex.y = 0.0;
        this.ey.y = 0.0;
    }
    b2Mat22.prototype.GetInverse = function() {
        var a = this.ex.x;
        var b = this.ey.x;
        var c = this.ex.y;
        var d = this.ey.y;
        var det = a * d - b * c;
        if (det != 0.0) {
            det = 1.0 / det;
        }
        var out = new b2Mat22();
        out.ex.x = det * d;
        out.ey.x = (-det * b);
        out.ex.y = (-det * c);
        out.ey.y = det * a;
        return out;
    }
    b2Mat22.prototype.GetInverse2 = function(out) {
        var a = this.ex.x;
        var b = this.ey.x;
        var c = this.ex.y;
        var d = this.ey.y;
        var det = a * d - b * c;
        if (det != 0.0) {
            det = 1.0 / det;
        }
        out.ex.x = det * d;
        out.ey.x = (-det * b);
        out.ex.y = (-det * c);
        out.ey.y = det * a;
    }
    b2Mat22.prototype.Solve = function(b) {
        var a11 = this.ex.x;
        var a12 = this.ey.x;
        var a21 = this.ex.y;
        var a22 = this.ey.y;
        var det = a11 * a22 - a12 * a21;
        if (det != 0.0) {
            det = 1.0 / det;
        }
        var out = new b2Vec2();
        out.x = det * (a22 * b.x - a12 * b.y);
        out.y = det * (a11 * b.y - a21 * b.x);
        return out;
    }
    b2Mat33.b2Mat33 = function() {
        this.ex = new b2Vec3();
        this.ey = new b2Vec3();
        this.ez = new b2Vec3();
    };
    b2Mat33.prototype.b2Mat33 = function(c1, c2, c3) {
        if (c1 != undefined && c2 != undefined && c3 != undefined) {
            this.ex.SetV(c1);
            this.ey.SetV(c2);
            this.ez.SetV(c3);
        }
    }
    b2Mat33.prototype.Copy = function() {
        return new b2Mat33(this.ex, this.ey, this.ez);
    }
    b2Mat33.prototype.SetM = function(m) {
        this.ex.SetV(m.ex);
        this.ey.SetV(m.ey);
        this.ez.SetV(m.ez);
    }
    b2Mat33.prototype.SetZero = function() {
        this.ex.x = 0.0;
        this.ey.x = 0.0;
        this.ez.x = 0.0;
        this.ex.y = 0.0;
        this.ey.y = 0.0;
        this.ez.y = 0.0;
        this.ex.z = 0.0;
        this.ey.z = 0.0;
        this.ez.z = 0.0;
    }
    b2Mat33.prototype.Solve22 = function(b) {
        var a11 = this.ex.x;
        var a12 = this.ey.x;
        var a21 = this.ex.y;
        var a22 = this.ey.y;
        var det = a11 * a22 - a12 * a21;
        if (det != 0.0) {
            det = 1.0 / det;
        }
        var out = new b2Vec2();
        out.x = det * (a22 * b.x - a12 * b.y);
        out.y = det * (a11 * b.y - a21 * b.x);
        return out;
    }
    b2Mat33.prototype.Solve33 = function(b) {
        var det = b2Math.DotVV3(this.ex, b2Math.CrossVV3(this.ey, this.ez));
        if (det != 0.0) {
            det = 1.0 / det;
        }
        var x = new b2Vec3();
        x.x = det * b2Math.DotVV3(b, b2Math.CrossVV3(this.ey, this.ez));
        x.y = det * b2Math.DotVV3(this.ex, b2Math.CrossVV3(b, this.ez));
        x.z = det * b2Math.DotVV3(this.ex, b2Math.CrossVV3(this.ey, b));
        return x;
    }
    b2Mat33.prototype.GetInverse22 = function(outM) {
        var a = this.ex.x;
        var b = this.ey.x;
        var c = this.ex.y;
        var d = this.ey.y;
        var det = a * d - b * c;
        if (det != 0.0) {
            det = 1.0 / det;
        }

        outM.ex.x = det * d;
        outM.ey.x = -det * b;
        outM.ex.z = 0.0;
        outM.ex.y = -det * c;
        outM.ey.y = det * a;
        outM.ey.z = 0.0;
        outM.ez.x = 0.0;
        outM.ez.y = 0.0;
        outM.ez.z = 0.0;
    }
    b2Mat33.prototype.GetSymInverse33 = function(outM) {
        var det = b2Math.DotVV3(this.ex, b2Math.CrossVV3(this.ey, this.ez));
        if (det != 0.0) {
            det = 1.0 / det;
        }
        var a11 = this.ex.x;
        var a12 = this.ey.x;
        var a13 = this.ez.x;
        var a22 = this.ey.y;
        var a23 = this.ez.y;
        var a33 = this.ez.z;

        outM.ex.x = det * (a22 * a33 - a23 * a23);
        outM.ex.y = det * (a13 * a23 - a12 * a33);
        outM.ex.z = det * (a12 * a23 - a13 * a22);

        outM.ey.x = outM.ex.y;
        outM.ey.y = det * (a11 * a33 - a13 * a13);
        outM.ey.z = det * (a13 * a12 - a11 * a23);

        outM.ez.x = outM.ex.z;
        outM.ez.y = outM.ey.z;
        outM.ez.z = det * (a11 * a22 - a12 * a12);
    }
    b2Rot.b2Rot = function() {};
    b2Rot.prototype.b2Rot = function(angle) {
        this.s = Math.sin(angle);
        this.c = Math.cos(angle);
    }
    b2Rot.prototype.Set = function(angle) {
        this.s = Math.sin(angle);
        this.c = Math.cos(angle);
    }
    b2Rot.prototype.SetIdentity = function() {
        this.s = 0;
        this.c = 1;
    }
    b2Rot.prototype.GetAngle = function() {
        return Math.atan2(this.s, this.c);
    }
    b2Rot.prototype.GetXAxis = function() {
        return new b2Vec2(this.c, this.s);
    }
    b2Rot.prototype.GetYAxis = function() {
        return new b2Vec2(-this.s, this.c);
    }
    b2Math.b2Math = function() {};
    b2Math.IsValid = function(x) {
        if (x === undefined) return false;
        if (isNaN(x)) return false;
        return isFinite(x);
    }
    b2Math.Dot = function(a, b) {
        if (a.z != undefined && b.z != undefined) {
            return b2Math.DotVV3(a, b);
        }
        return b2Math.DotVV(a, b);
    }
    b2Math.DotVV = function(a, b) {
        return a.x * b.x + a.y * b.y;
    }
    b2Math.DotVV3 = function(a, b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }
    b2Math.Cross = function(a, b) {
        if (a instanceof b2Vec2 && b instanceof b2Vec2) {
            return b2Math.CrossVV(a, b);
        }
        if (a instanceof b2Vec2 && typeof(b) === 'number') {
            return b2Math.CrossVF(a, b);
        }
        if (b instanceof b2Vec2 && typeof(a) === 'number') {
            return b2Math.CrossFV(a, b);
        }
        if (a instanceof b2Vec3 && b instanceof b2Vec3) {
            return b2Math.CrossVV3(a, b);
        }
        b2Settings.b2Assert(0);
    }
    b2Math.CrossVV = function(a, b) {
        //b2Settings.b2Assert(a instanceof b2Vec2 && b instanceof b2Vec2);
        return a.x * b.y - a.y * b.x;
    }
    b2Math.CrossVF = function(a, s) {
        //b2Settings.b2Assert(a instanceof b2Vec2 && typeof(s) === 'number');
        var v = new b2Vec2(s * a.y, (-s * a.x));
        return v;
    }
    b2Math.CrossFV = function(s, a) {
        //b2Settings.b2Assert(a instanceof b2Vec2 && typeof(s) === 'number');
        var v = new b2Vec2((-s * a.y), s * a.x);
        return v;
    }
    b2Math.CrossVV3 = function(a, b) {
        //b2Settings.b2Assert(a instanceof b2Vec3 && b instanceof b2Vec3);
        return new b2Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
    }
    b2Math.Mul = function(a, b) {
        if (typeof(a) === 'number' && b instanceof b2Vec2) {
            return b2Math.MulFV(a, b);
        }
        if (typeof(a) === 'number' && b instanceof b2Vec3) {
            return b2Math.MulFV3(a, b);
        }
        if (a instanceof b2Mat22 && b instanceof b2Vec2) {
            return b2Math.MulMV(a, b);
        }
        if (a instanceof b2Mat22 && b instanceof b2Mat22) {
            return b2Math.MulMM(a, b);
        }
        if (a instanceof b2Mat33 && b instanceof b2Vec3) {
            return b2Math.MulMV3(a, b);
        }
        if (a instanceof b2Mat33 && b instanceof b2Vec3) {
            return b2Math.MulMV3(a, b);
        }
        if (a instanceof b2Rot && b instanceof b2Rot) {
            return b2Math.MulRR(a, b);
        }
        if (a instanceof b2Rot && b instanceof b2Vec2) {
            return b2Math.MulRV(a, b);
        }
        if (a instanceof b2Transform && b instanceof b2Vec2) {
            return b2Math.MulXV(a, b);
        }
        if (a instanceof b2Transform && b instanceof b2Transform) {
            return b2Math.MulXX(a, b);
        }
        b2Settings.b2Assert(0);
    }
    b2Math.MulFV = function(s, a) {
        //b2Settings.b2Assert(typeof(s) === 'number' && a instanceof b2Vec2);
        return new b2Vec2(s * a.x, s * a.y);
    }
    b2Math.MulFV3 = function(s, a) {
        //b2Settings.b2Assert(typeof(s) === 'number' && a instanceof b2Vec3);
        return new b2Vec3(s * a.x, s * a.y, s * a.z);
    }
    b2Math.MulMV = function(A, v) {
        //b2Settings.b2Assert(A instanceof b2Mat22 && v instanceof b2Vec2);
        return new b2Vec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y);
    }
    b2Math.MulMM = function(A, B) {
        //b2Settings.b2Assert(A instanceof b2Mat22 && B instanceof b2Mat22);
        var C = new b2Mat22(b2Math.MulMV(A, B.ex), b2Math.MulMV(A, B.ey));
        return C;
    }
    b2Math.MulMV3 = function(A, v) {
        //b2Settings.b2Assert(A instanceof b2Mat33 && v instanceof b2Vec3);
        return new b2Vec3(A.ex.x * v.x + A.ey.x * v.y + A.ez.x * v.z, A.ex.y * v.x + A.ey.y * v.y + A.ez.y * v.z, A.ex.z * v.x + A.ey.z * v.y + A.ez.z * v.z);
    }
    b2Math.MulRV = function(R, v) {
        //b2Settings.b2Assert(R instanceof b2Rot && v instanceof b2Vec2);
        var u = new b2Vec2(R.c * v.x - R.s * v.y, R.s * v.x + R.c * v.y);
        return u;
    }
    b2Math.MulRR = function(q, r) {
        //b2Settings.b2Assert(q instanceof b2Rot && r instanceof b2Rot);
        var qr = new b2Rot();
        qr.s = q.s * r.c + q.c * r.s;
        qr.c = q.c * r.c - q.s * r.s;
        return qr;
    }
    b2Math.MulT = function(a, b) {
        if (a instanceof b2Mat22 && b instanceof b2Vec2) {
            return b2Math.MulTMV(a, b);
        }
        if (a instanceof b2Mat22 && b instanceof b2Mat22) {
            return b2Math.MulTMM(a, b);
        }
        if (a instanceof b2Rot && b instanceof b2Rot) {
            return b2Math.MulTRR(a, b);
        }
        if (a instanceof b2Rot && b instanceof b2Vec2) {
            return b2Math.MulTRV(a, b);
        }
        if (a instanceof b2Transform && b instanceof b2Vec2) {
            return b2Math.MulTXV(a, b);
        }
        if (a instanceof b2Transform && b instanceof b2Transform) {
            return b2Math.MulTXX(a, b);
        }
        b2Settings.b2Assert(0);
    }
    b2Math.MulTMV = function(A, v) {
        //b2Settings.b2Assert(A instanceof b2Mat22 && v instanceof b2Vec2);
        var u = new b2Vec2(b2Math.DotVV(v, A.ex), b2Math.DotVV(v, A.ey));
        return u;
    }
    b2Math.MulTRV = function(R, v) {
        //b2Settings.b2Assert(R instanceof b2Rot && v instanceof b2Vec2);
        return new b2Vec2(R.c * v.x + R.s * v.y, -R.s * v.x + R.c * v.y);
    }
    b2Math.MulTRR = function(q, r) {
        //b2Settings.b2Assert(q instanceof b2Rot && r instanceof b2Rot);
        var qr = new b2Rot();
        qr.s = q.c * r.s - q.s * r.c;
        qr.c = q.c * r.c + q.s * r.s;
        return qr;
    }
    b2Math.MulTMM = function(A, B) {
        //b2Settings.b2Assert(A instanceof b2Mat22 && B instanceof b2Mat22);
        var c1 = new b2Vec2(b2Math.DotVV(A.ex, B.ex), b2Math.DotVV(A.ey, B.ex));
        var c2 = new b2Vec2(b2Math.DotVV(A.ex, B.ey), b2Math.DotVV(A.ey, B.ey));
        var C = new b2Mat22(c1, c2);
        return C;
    }
    b2Math.MulXV = function(T, v) {
        //b2Settings.b2Assert(T instanceof b2Transform && v instanceof b2Vec2);
        var x = (T.q.c * v.x - T.q.s * v.y) + T.p.x;
        var y = (T.q.s * v.x + T.q.c * v.y) + T.p.y;
        return new b2Vec2(x, y);
    }
    b2Math.MulXX = function(A, B) {
        //b2Settings.b2Assert(A instanceof b2Transform && B instanceof b2Transform);
        var C = new b2Transform();
        C.q = b2Math.MulRR(A.q, B.q);
        C.p = b2Math.AddVV(b2Math.MulRV(A.q, B.p), A.p);
        return C;
    }
    b2Math.MulTXV = function(T, v) {
        //b2Settings.b2Assert(T instanceof b2Transform && v instanceof b2Vec2);
        var px = v.x - T.p.x;
        var py = v.y - T.p.y;
        var x = (T.q.c * px + T.q.s * py);
        var y = (-T.q.s * px + T.q.c * py);
        return new b2Vec2(x, y);
    }
    b2Math.MulTXX = function(A, B) {
        //b2Settings.b2Assert(A instanceof b2Transform && B instanceof b2Transform);
        var C = new b2Transform();
        C.q = b2Math.MulTRR(A.q, B.q);
        C.p = b2Math.MulTRV(A.q, b2Math.SubtractVV(B.p, A.p));
        return C;
    }
    b2Math.AddVV = function(a, b) {
        //b2Settings.b2Assert(a instanceof b2Vec2 && b instanceof b2Vec2);
        var v = new b2Vec2(a.x + b.x, a.y + b.y);
        return v;
    }
    b2Math.AddVV3 = function(a, b) {
        //b2Settings.b2Assert(a instanceof b2Vec3 && b instanceof b2Vec3);
        var v = new b2Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
        return v;
    }
    b2Math.SubtractVV = function(a, b) {
        //b2Settings.b2Assert(a instanceof b2Vec2 && b instanceof b2Vec2);
        var v = new b2Vec2(a.x - b.x, a.y - b.y);
        return v;
    }
    b2Math.SubtractVV3 = function(a, b) {
        //b2Settings.b2Assert(a instanceof b2Vec3 && b instanceof b2Vec3);
        var v = new b2Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
        return v;
    }
    b2Math.Distance = function(a, b) {
        //b2Settings.b2Assert(a instanceof b2Vec2 && b instanceof b2Vec2);
        var cX = a.x - b.x;
        var cY = a.y - b.y;
        return Math.sqrt(cX * cX + cY * cY);
    }
    b2Math.DistanceSquared = function(a, b) {
        //b2Settings.b2Assert(a instanceof b2Vec2 && b instanceof b2Vec2);
        var cX = a.x - b.x;
        var cY = a.y - b.y;
        return (cX * cX + cY * cY);
    }
    b2Math.AddMM = function(A, B) {
        //b2Settings.b2Assert(A instanceof b2Mat22 && B instanceof b2Mat22);
        var C = new b2Mat22(b2Math.AddVV(A.ex, B.ex), b2Math.AddVV(A.ey, B.ey));
        return C;
    }
    b2Math.Abs = function(a) {
        //b2Settings.b2Assert(typeof(a) === 'number');
        return a > 0.0 ? a : (-a);
    }
    b2Math.AbsV = function(a) {
        //b2Settings.b2Assert(a instanceof b2Vec2);
        var b = new b2Vec2(b2Math.Abs(a.x), b2Math.Abs(a.y));
        return b;
    }
    b2Math.AbsM = function(A) {
        //b2Settings.b2Assert(A instanceof b2Mat22);
        var B = new b2Mat22(b2Math.AbsV(A.ex), b2Math.AbsV(A.ey));
        return B;
    }
    b2Math.Min = function(a, b) {
        //b2Settings.b2Assert(typeof(a) === 'number' && typeof(b) === 'number');
        return a < b ? a : b;
    }
    b2Math.MinV = function(a, b) {
        //b2Settings.b2Assert(a instanceof b2Vec2 && b instanceof b2Vec2);
        var c = new b2Vec2(b2Math.Min(a.x, b.x), b2Math.Min(a.y, b.y));
        return c;
    }
    b2Math.Max = function(a, b) {
        //b2Settings.b2Assert(typeof(a) === 'number' && typeof(b) === 'number');
        return a > b ? a : b;
    }
    b2Math.MaxV = function(a, b) {
        //b2Settings.b2Assert(a instanceof b2Vec2 && b instanceof b2Vec2);
        var c = new b2Vec2(b2Math.Max(a.x, b.x), b2Math.Max(a.y, b.y));
        return c;
    }
    b2Math.Clamp = function(a, low, high) {
        //b2Settings.b2Assert(typeof(a) === 'number' && typeof(high) === 'number' && typeof(low) === 'number');
        return a < low ? low : a > high ? high : a;
    }
    b2Math.ClampV = function(a, low, high) {
        //b2Settings.b2Assert(a instanceof b2Vec2 && low instanceof b2Vec2 && high instanceof b2Vec2);
        return b2Math.MaxV(low, b2Math.MinV(a, high));
    }
    b2Math.Swap = function(a, b) {
        var tmp = a[0];
        a[0] = b[0];
        b[0] = tmp;
    }
    b2Math.Random = function() {
        return Math.random() * 2 - 1;
    }
    b2Math.RandomRange = function(lo, hi) {
        if (lo === undefined) lo = 0;
        if (hi === undefined) hi = 0;
        var r = Math.random();
        r = (hi - lo) * r + lo;
        return r;
    }
    b2Math.NextPowerOfTwo = function(x) {
        x |= (x >> 1) & 0x7FFFFFFF;
        x |= (x >> 2) & 0x3FFFFFFF;
        x |= (x >> 4) & 0x0FFFFFFF;
        x |= (x >> 8) & 0x00FFFFFF;
        x |= (x >> 16) & 0x0000FFFF;
        return x + 1;
    }
    b2Math.IsPowerOfTwo = function(x) {
        var result = x > 0 && (x & (x - 1)) == 0;
        return result;
    }
    Box2D.postDefs.push(function() {
        Box2D.Common.Math.b2Math.b2Vec2_zero = new b2Vec2(0.0, 0.0);
        Box2D.Common.Math.b2Math.b2Mat22_identity = new b2Mat22(new b2Vec2(1.0, 0.0), new b2Vec2(0.0, 1.0));
        Box2D.Common.Math.b2Math.b2Transform_identity = new b2Transform(b2Math.b2Vec2_zero, b2Math.b2Mat22_identity);
    });
    b2Sweep.b2Sweep = function() {
        this.localCenter = new b2Vec2();
        this.c0 = new b2Vec2;
        this.c = new b2Vec2();
        this.a = 0;
        this.a0 = 0;
        this.alpha0 = 0;
    };
    b2Sweep.prototype.Set = function(other) {
        this.localCenter.SetV(other.localCenter);
        this.c0.SetV(other.c0);
        this.c.SetV(other.c);
        this.a0 = other.a0;
        this.a = other.a;
        this.alpha0 = other.alpha0;
    }
    b2Sweep.prototype.Copy = function() {
        var copy = new b2Sweep();
        copy.Set(this);
        return copy;
    }
    b2Sweep.prototype.GetTransform = function(xf, alpha) {
        xf.p.x = (1.0 - alpha) * this.c0.x + alpha * this.c.x;
        xf.p.y = (1.0 - alpha) * this.c0.y + alpha * this.c.y;
        var angle = (1.0 - alpha) * this.a0 + alpha * this.a;
        xf.q.Set(angle);
        var tRot = xf.q;
        xf.p.x -= (tRot.c * this.localCenter.x - tRot.s * this.localCenter.y);
        xf.p.y -= (tRot.s * this.localCenter.x + tRot.c * this.localCenter.y);
    }
    b2Sweep.prototype.Advance = function(t) {
        if (this.alpha0 < t && 1.0 - this.alpha0 > Number.MIN_VALUE) {
            var alpha = (t - this.alpha0) / (1.0 - this.alpha0);
            this.c0.x = (1.0 - alpha) * this.c0.x + alpha * this.c.x;
            this.c0.y = (1.0 - alpha) * this.c0.y + alpha * this.c.y;
            this.a0 = (1.0 - alpha) * this.a0 + alpha * this.a;
            this.alpha0 = t;
        }
    }
    b2Sweep.prototype.Normalize = function(t) {
        var twoPi = 2.0 * b2Settings.b2_pi;
        var d =  twoPi * parseInt(this.a0 / twoPi);
        this.a0 -= d;
        this.a -= d;
    }
    b2Transform.b2Transform = function() {
        this.p = new b2Vec2;
        this.q = new b2Rot();
    };
    b2Transform.prototype.b2Transform = function(pos, r) {
        if (pos === undefined) pos = null;
        if (r === undefined) r = null;
        if (pos) {
            this.p.Set(pos.x, pos.y);
            this.q.s = r.s;
            this.q.c = r.c;
        }
    }
    b2Transform.prototype.Copy = function() {
        var copy = new b2Transform();
        copy.p.SetV(this.p);
        copy.q.s = this.q.s;
        copy.q.c = this.q.c;
        return copy;
    }
/*   b2Transform.prototype.Initialize = function (pos, r) {
      this.position.SetV(pos);
      this.R.SetM(r);
   }
   */
    b2Transform.prototype.SetIdentity = function() {
        this.p.SetZero();
        this.q.SetIdentity();
    }
    b2Transform.prototype.Set = function(pos, angle) {
        this.p.Set(pos.x, pos.y);
        this.q.Set(angle);
    }
/*   b2Transform.prototype.GetAngle = function () {
      return Math.atan2(this.R.ex.y, this.R.ex.x);
   }
   */
    b2Vec2.b2Vec2 = function() {
        this.x = 0;
        this.y = 0;
    };
    b2Vec2.prototype.b2Vec2 = function(x_, y_) {
        if (x_ === undefined) x_ = 0;
        if (y_ === undefined) y_ = 0;
        this.x = x_;
        this.y = y_;
    }
    b2Vec2.prototype.SetZero = function() {
        this.x = 0.0;
        this.y = 0.0;
    }
    b2Vec2.prototype.Set = function(x_, y_) {
        this.x = x_;
        this.y = y_;
    }
    b2Vec2.prototype.SetV = function(v) {
        this.x = v.x;
        this.y = v.y;
    }
   b2Vec2.prototype.GetNegative = function () {
      return new b2Vec2((-this.x), (-this.y));
   }
   /*
    b2Vec2.prototype.NegativeSelf = function() {
        this.x = (-this.x);
        this.y = (-this.y);
    }
   b2Vec2.Make = function (x_, y_) {
      if (x_ === undefined) x_ = 0;
      if (y_ === undefined) y_ = 0;
      return new b2Vec2(x_, y_);
   }
   */
    b2Vec2.prototype.Copy = function () {
        return new b2Vec2(this.x, this.y);
    }
    b2Vec2.prototype.Add = function(v) {
        this.x += v.x;
        this.y += v.y;
    }
    b2Vec2.prototype.Subtract = function(v) {
        this.x -= v.x;
        this.y -= v.y;
    }
    b2Vec2.prototype.Multiply = function(a) {
        this.x *= a;
        this.y *= a;
    }
/*
   b2Vec2.prototype.MulM = function (A) {
      var tX = this.x;
      this.x = A.ex.x * tX + A.ey.x * this.y;
      this.y = A.ex.y * tX + A.ey.y * this.y;
   }
   b2Vec2.prototype.MulTM = function (A) {
      var tX = b2Math.Dot(this, A.ex);
      this.y = b2Math.Dot(this, A.ey);
      this.x = tX;
   }
   b2Vec2.prototype.CrossVF = function (s) {
      if (s === undefined) s = 0;
      var tX = this.x;
      this.x = s * this.y;
      this.y = (-s * tX);
   }
   b2Vec2.prototype.CrossFV = function (s) {
      if (s === undefined) s = 0;
      var tX = this.x;
      this.x = (-s * this.y);
      this.y = s * tX;
   }
   b2Vec2.prototype.MinV = function (b) {
      this.x = this.x < b.x ? this.x : b.x;
      this.y = this.y < b.y ? this.y : b.y;
   }
   b2Vec2.prototype.MaxV = function (b) {
      this.x = this.x > b.x ? this.x : b.x;
      this.y = this.y > b.y ? this.y : b.y;
   }
   b2Vec2.prototype.Abs = function () {
      if (this.x < 0) this.x = (-this.x);
      if (this.y < 0) this.y = (-this.y);
   }
   */
    b2Vec2.prototype.Length = function() {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }
    b2Vec2.prototype.LengthSquared = function() {
        return (this.x * this.x + this.y * this.y);
    }
    b2Vec2.prototype.Normalize = function() {
        var length = Math.sqrt(this.x * this.x + this.y * this.y);
        if (length < Number.MIN_VALUE) {
            return 0.0;
        }
        var invLength = 1.0 / length;
        this.x *= invLength;
        this.y *= invLength;
        return length;
    }
    b2Vec2.prototype.IsValid = function() {
        return b2Math.IsValid(this.x) && b2Math.IsValid(this.y);
    }
    b2Vec2.prototype.Skew = function() {
        return new b2Vec2(-this.y, this.x);
    }
    b2Vec3.b2Vec3 = function() {};
    b2Vec3.prototype.b2Vec3 = function(x, y, z) {
        if (x === undefined) x = 0;
        if (y === undefined) y = 0;
        if (z === undefined) z = 0;
        this.x = x;
        this.y = y;
        this.z = z;
    }
    b2Vec3.prototype.SetZero = function() {
        this.x = this.y = this.z = 0.0;
    }
    b2Vec3.prototype.Set = function(x, y, z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
    b2Vec3.prototype.SetV = function(v) {
        this.x = v.x;
        this.y = v.y;
        this.z = v.z;
    }
    b2Vec3.prototype.GetNegative = function() {
        return new b2Vec3((-this.x), (-this.y), (-this.z));
    }
    b2Vec3.prototype.NegativeSelf = function() {
        this.x = (-this.x);
        this.y = (-this.y);
        this.z = (-this.z);
    }
    b2Vec3.prototype.Copy = function() {
        return new b2Vec3(this.x, this.y, this.z);
    }
    b2Vec3.prototype.Add = function(v) {
        this.x += v.x;
        this.y += v.y;
        this.z += v.z;
    }
    b2Vec3.prototype.Subtract = function(v) {
        this.x -= v.x;
        this.y -= v.y;
        this.z -= v.z;
    }
    b2Vec3.prototype.Multiply = function(a) {
        this.x *= a;
        this.y *= a;
        this.z *= a;
    }
})();
(function() {
    var b2Mat22 = Box2D.Common.Math.b2Mat22,
        b2Mat33 = Box2D.Common.Math.b2Mat33,
        b2Rot = Box2D.Common.Math.b2Rot,
        b2Math = Box2D.Common.Math.b2Math,
        b2Sweep = Box2D.Common.Math.b2Sweep,
        b2Transform = Box2D.Common.Math.b2Transform,
        b2Vec2 = Box2D.Common.Math.b2Vec2,
        b2Vec3 = Box2D.Common.Math.b2Vec3,
        b2Color = Box2D.Common.b2Color,
        b2Settings = Box2D.Common.b2Settings,
        b2Timer = Box2D.Common.b2Timer,
        b2AABB = Box2D.Collision.b2AABB,
        b2Collision = Box2D.Collision.b2Collision,
        b2ContactID = Box2D.Collision.b2ContactID,
        b2Distance = Box2D.Collision.b2Distance,
        b2DistanceInput = Box2D.Collision.b2DistanceInput,
        b2DistanceOutput = Box2D.Collision.b2DistanceOutput,
        b2DistanceProxy = Box2D.Collision.b2DistanceProxy,
        b2DynamicTree = Box2D.Collision.b2DynamicTree,
        b2BroadPhase = Box2D.Collision.b2BroadPhase,
        b2TreeNode = Box2D.Collision.b2TreeNode,
        b2Pair = Box2D.Collision.b2Pair,
        b2Manifold = Box2D.Collision.b2Manifold,
        b2ManifoldPoint = Box2D.Collision.b2ManifoldPoint,
        b2RayCastInput = Box2D.Collision.b2RayCastInput,
        b2RayCastOutput = Box2D.Collision.b2RayCastOutput,
        b2SeparationFunction = Box2D.Collision.b2SeparationFunction,
        b2Simplex = Box2D.Collision.b2Simplex,
        b2SimplexCache = Box2D.Collision.b2SimplexCache,
        b2SimplexVertex = Box2D.Collision.b2SimplexVertex,
        b2TOIInput = Box2D.Collision.b2TOIInput,
        b2TOIOutput = Box2D.Collision.b2TOIOutput,
        b2WorldManifold = Box2D.Collision.b2WorldManifold,
        b2ClipVertex = Box2D.Collision.b2ClipVertex,
        b2ContactFeature = Box2D.Collision.b2ContactFeature,
        IBroadPhase = Box2D.Collision.IBroadPhase,
        b2CircleShape = Box2D.Collision.Shapes.b2CircleShape,
        b2EdgeShape = Box2D.Collision.Shapes.b2EdgeShape,
        b2ChainShape = Box2D.Collision.Shapes.b2ChainShape,
        b2MassData = Box2D.Collision.Shapes.b2MassData,
        b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape,
        b2Shape = Box2D.Collision.Shapes.b2Shape,
        b2Body = Box2D.Dynamics.b2Body,
        b2BodyDef = Box2D.Dynamics.b2BodyDef,
        b2ContactFilter = Box2D.Dynamics.b2ContactFilter,
        b2ContactImpulse = Box2D.Dynamics.b2ContactImpulse,
        b2ContactListener = Box2D.Dynamics.b2ContactListener,
        b2ContactManager = Box2D.Dynamics.b2ContactManager,
        b2Draw = Box2D.Dynamics.b2Draw,
        b2DestructionListener = Box2D.Dynamics.b2DestructionListener,
        b2Filter = Box2D.Dynamics.b2Filter,
        b2Fixture = Box2D.Dynamics.b2Fixture,
        b2FixtureDef = Box2D.Dynamics.b2FixtureDef,
        b2FixtureProxy = Box2D.Dynamics.b2FixtureProxy,
        b2Island = Box2D.Dynamics.b2Island,
        b2Position = Box2D.Dynamics.b2Position,
        b2Profile = Box2D.Dynamics.b2Profile,
        b2SolverData = Box2D.Dynamics.b2SolverData,
        b2TimeStep = Box2D.Dynamics.b2TimeStep,
        b2Velocity = Box2D.Dynamics.b2Velocity,
        b2World = Box2D.Dynamics.b2World,
        b2WorldQueryWrapper = Box2D.Dynamics.b2WorldQueryWrapper,
        b2WorldRayCastWrapper = Box2D.Dynamics.b2WorldRayCastWrapper,
        b2ChainAndCircleContact = Box2D.Dynamics.Contacts.b2ChainAndCircleContact,
        b2ChainAndPolygonContact = Box2D.Dynamics.Contacts.b2ChainAndPolygonContact,
        b2CircleContact = Box2D.Dynamics.Contacts.b2CircleContact,
        b2Contact = Box2D.Dynamics.Contacts.b2Contact,
        b2ContactEdge = Box2D.Dynamics.Contacts.b2ContactEdge,
        b2ContactPositionConstraint = Box2D.Dynamics.Contacts.b2ContactPositionConstraint,
        b2ContactRegister = Box2D.Dynamics.Contacts.b2ContactRegister,
        b2ContactSolver = Box2D.Dynamics.Contacts.b2ContactSolver,
        b2ContactSolverDef = Box2D.Dynamics.Contacts.b2ContactSolverDef,
        b2ContactVelocityConstraint = Box2D.Dynamics.Contacts.b2ContactVelocityConstraint,
        b2EdgeAndCircleContact = Box2D.Dynamics.Contacts.b2EdgeAndCircleContact,
        b2PolygonAndCircleContact = Box2D.Dynamics.Contacts.b2PolygonAndCircleContact,
        b2EdgeAndPolygonContact = Box2D.Dynamics.Contacts.b2EdgeAndPolygonContact,
        b2PolygonContact = Box2D.Dynamics.Contacts.b2PolygonContact,
        b2PositionSolverManifold = Box2D.Dynamics.Contacts.b2PositionSolverManifold,
        b2VelocityConstraintPoint = Box2D.Dynamics.Contacts.b2VelocityConstraintPoint,
        b2DistanceJoint = Box2D.Dynamics.Joints.b2DistanceJoint,
        b2DistanceJointDef = Box2D.Dynamics.Joints.b2DistanceJointDef,
        b2FrictionJoint = Box2D.Dynamics.Joints.b2FrictionJoint,
        b2FrictionJointDef = Box2D.Dynamics.Joints.b2FrictionJointDef,
        b2GearJoint = Box2D.Dynamics.Joints.b2GearJoint,
        b2GearJointDef = Box2D.Dynamics.Joints.b2GearJointDef,
        b2Jacobian = Box2D.Dynamics.Joints.b2Jacobian,
        b2Joint = Box2D.Dynamics.Joints.b2Joint,
        b2JointDef = Box2D.Dynamics.Joints.b2JointDef,
        b2JointEdge = Box2D.Dynamics.Joints.b2JointEdge,
        b2MouseJoint = Box2D.Dynamics.Joints.b2MouseJoint,
        b2MouseJointDef = Box2D.Dynamics.Joints.b2MouseJointDef,
        b2PrismaticJoint = Box2D.Dynamics.Joints.b2PrismaticJoint,
        b2PrismaticJointDef = Box2D.Dynamics.Joints.b2PrismaticJointDef,
        b2PulleyJoint = Box2D.Dynamics.Joints.b2PulleyJoint,
        b2PulleyJointDef = Box2D.Dynamics.Joints.b2PulleyJointDef,
        b2RevoluteJoint = Box2D.Dynamics.Joints.b2RevoluteJoint,
        b2RevoluteJointDef = Box2D.Dynamics.Joints.b2RevoluteJointDef,
        b2RopeJoint = Box2D.Dynamics.Joints.b2RopeJoint,
        b2RopeJointDef = Box2D.Dynamics.Joints.b2RopeJointDef,
        b2WeldJoint = Box2D.Dynamics.Joints.b2WeldJoint,
        b2WeldJointDef = Box2D.Dynamics.Joints.b2WeldJointDef,
        b2WheelJoint = Box2D.Dynamics.Joints.b2WheelJoint,
        b2WheelJointDef = Box2D.Dynamics.Joints.b2WheelJointDef,
        b2Rope = Box2D.Rope.b2Rope,
        b2RopeDef = Box2D.Rope.b2RopeDef;

    b2Body.b2Body = function() {
        this.m_xf = new b2Transform();
        this.m_sweep = new b2Sweep();
        this.m_linearVelocity = new b2Vec2();
        this.m_force = new b2Vec2();
    };
/*   b2Body.prototype.connectEdges = function (s1, s2, angle1) {
      if (angle1 === undefined) angle1 = 0;
      var angle2 = Math.atan2(s2.GetDirectionVector().y, s2.GetDirectionVector().x);
      var coreOffset = Math.tan((angle2 - angle1) * 0.5);
      var core = b2Math.MulFV(coreOffset, s2.GetDirectionVector());
      core = b2Math.SubtractVV(core, s2.GetNormalVector());
      core = b2Math.MulFV(b2Settings.b2_toiSlop, core);
      core = b2Math.AddVV(core, s2.GetVertex1());
      var cornerDir = b2Math.AddVV(s1.GetDirectionVector(), s2.GetDirectionVector());
      cornerDir.Normalize();
      var convex = b2Math.Dot(s1.GetDirectionVector(), s2.GetNormalVector()) > 0.0;
      s1.SetNextEdge(s2, core, cornerDir, convex);
      s2.SetPrevEdge(s1, core, cornerDir, convex);
      return angle2;
   }
   */
    b2Body.prototype.CreateFixture = function(arg1, arg2) {
        if (arg1 instanceof b2Shape) {
            return this.CreateFixture2(arg1, arg2);
        }
        var def = arg1;
        b2Settings.b2Assert(this.m_world.IsLocked() == false);
        if (this.m_world.IsLocked() == true) {
            return null;
        }
        var fixture = new b2Fixture();
        fixture.Create(this.m_world.m_blockAllocator, this, def);

        if (this.m_flags & b2Body.e_activeFlag) {
            var broadPhase = this.m_world.m_contactManager.m_broadPhase;
            fixture.CreateProxies(broadPhase, this.m_xf);
        }
        fixture.m_next = this.m_fixtureList;
        this.m_fixtureList = fixture;
        ++this.m_fixtureCount;
        fixture.m_body = this;
        if (fixture.m_density > 0.0) {
            this.ResetMassData();
        }
        this.m_world.m_flags |= b2World.e_newFixture;
        return fixture;
    }
    b2Body.prototype.CreateFixture2 = function(shape, density) {
        var def = new b2FixtureDef();
        def.shape = shape;
        def.density = density;
        return this.CreateFixture(def);
    }
    b2Body.prototype.DestroyFixture = function(fixture) {
        if (this.m_world.IsLocked() == true) {
            return;
        }
//        b2Settings.b2Assert(fixture.m_body == this);
//        b2Settings.b2Assert(this.m_fixtureCount > 0);
        var node = this.m_fixtureList;
        var ppF = null;
        var found = false;
        while (node != null) {
            if (node == fixture) {
                if (ppF) ppF.m_next = fixture.m_next;
                else this.m_fixtureList = fixture.m_next;
                found = true;
                break;
            }
            ppF = node;
            node = node.m_next;
        }
        var edge = this.m_contactList;
        while (edge) {
            var c = edge.contact;
            edge = edge.next;
            var fixtureA = c.GetFixtureA();
            var fixtureB = c.GetFixtureB();
            if (fixture == fixtureA || fixture == fixtureB) {
                this.m_world.m_contactManager.Destroy(c);
            }
        }
        if (this.m_flags & b2Body.e_activeFlag) {
            var broadPhase = this.m_world.m_contactManager.m_broadPhase;
            fixture.DestroyProxies(broadPhase);
        }
        fixture.Destroy();
        fixture.m_body = null;
        fixture.m_next = null;
        --this.m_fixtureCount;
        this.ResetMassData();
    }
    b2Body.prototype.SetTransform = function(position, angle) {
        if (this.m_world.IsLocked() == true) {
            return;
        }
        this.m_xf.Set(position, angle);
        this.m_sweep.c = b2Math.MulXV(this.m_xf, this.m_sweep.localCenter);
        this.m_sweep.a0 = this.m_sweep.a = angle;
        this.m_sweep.c0.SetV(this.m_sweep.c);

        var broadPhase = this.m_world.m_contactManager.m_broadPhase;
        for (var f = this.m_fixtureList; f; f = f.m_next) {
            f.Synchronize(broadPhase, this.m_xf, this.m_xf);
        }
        this.m_world.m_contactManager.FindNewContacts();
    }
    b2Body.prototype.GetTransform = function() {
        return this.m_xf;
    }
    b2Body.prototype.GetPosition = function() {
        return this.m_xf.p;
    }
    b2Body.prototype.GetAngle = function() {
        return this.m_sweep.a;
    }
/*   b2Body.prototype.SetAngle = function (angle) {
      if (angle === undefined) angle = 0;
      this.SetPositionAndAngle(this.GetPosition(), angle);
   }
   */
    b2Body.prototype.GetWorldCenter = function() {
        return this.m_sweep.c;
    }
    b2Body.prototype.GetLocalCenter = function() {
        return this.m_sweep.localCenter;
    }
    b2Body.prototype.SetLinearVelocity = function(v) {
        if (this.m_type == b2Body.b2_staticBody) {
            return;
        }
        if (b2Math.DotVV(v, v) > 0.0) {
            this.SetAwake(true);
        }
        this.m_linearVelocity.SetV(v);
    }
    b2Body.prototype.GetLinearVelocity = function() {
        return this.m_linearVelocity;
    }
    b2Body.prototype.SetAngularVelocity = function(omega) {
        if (this.m_type == b2Body.b2_staticBody) {
            return;
        }
        if (omega * omega > 0.0) {
            this.SetAwake(true);
        }
        this.m_angularVelocity = omega;
    }
    b2Body.prototype.GetAngularVelocity = function() {
        return this.m_angularVelocity;
    }
/*   b2Body.prototype.GetDefinition = function () {
      var bd = new b2BodyDef();
      bd.type = this.GetType();
      bd.allowSleep = (this.m_flags & b2Body.e_autoSleepFlag) == b2Body.e_autoSleepFlag;
      bd.angle = this.GetAngle();
      bd.angularDamping = this.m_angularDamping;
      bd.angularVelocity = this.m_angularVelocity;
      bd.fixedRotation = (this.m_flags & b2Body.e_fixedRotationFlag) == b2Body.e_fixedRotationFlag;
      bd.bullet = (this.m_flags & b2Body.e_bulletFlag) == b2Body.e_bulletFlag;
      bd.awake = (this.m_flags & b2Body.e_awakeFlag) == b2Body.e_awakeFlag;
      bd.linearDamping = this.m_linearDamping;
      bd.linearVelocity.SetV(this.GetLinearVelocity());
      bd.position = this.GetPosition();
      bd.userData = this.GetUserData();
      return bd;
   }
   */
    b2Body.prototype.ApplyForce = function(force, point) {
        if (this.m_type != b2Body.b2_dynamicBody) {
            return;
        }
        if (this.IsAwake() == false) {
            this.SetAwake(true);
        }
        this.m_force.x += force.x;
        this.m_force.y += force.y;
        this.m_torque += b2Math.CrossVV(b2Math.SubtractVV(point, this.m_sweep.c), force);
    }
    b2Body.prototype.ApplyForceToCenter = function(force) {
        if (this.m_type != b2Body.b2_dynamicBody) {
            return;
        }
        if (this.IsAwake() == false) {
            this.SetAwake(true);
        }
        this.m_force.x += force.x;
        this.m_force.y += force.y;
    }
    b2Body.prototype.ApplyTorque = function(torque) {
        if (this.m_type != b2Body.b2_dynamicBody) {
            return;
        }
        if (this.IsAwake() == false) {
            this.SetAwake(true);
        }
        this.m_torque += torque;
    }
    b2Body.prototype.ApplyLinearImpulse = function(impulse, point) {
        if (this.m_type != b2Body.b2_dynamicBody) {
            return;
        }
        if (this.IsAwake() == false) {
            this.SetAwake(true);
        }
        this.m_linearVelocity.x += this.m_invMass * impulse.x;
        this.m_linearVelocity.y += this.m_invMass * impulse.y;
        this.m_angularVelocity += this.m_invI * ((point.x - this.m_sweep.c.x) * impulse.y - (point.y - this.m_sweep.c.y) * impulse.x);
    }
    b2Body.prototype.ApplyAngularImpulse = function(impulse) {
        if (this.m_type != b2Body.b2_dynamicBody) {
            return;
        }
        if (this.IsAwake() == false) {
            this.SetAwake(true);
        }
        this.m_angularVelocity += this.m_invI * impulse;
    }
/*
   b2Body.prototype.Split = function (callback) {
      var linearVelocity = this.GetLinearVelocity().Copy();
      var angularVelocity = this.GetAngularVelocity();
      var center = this.GetWorldCenter();
      var body1 = this;
      var body2 = this.m_world.CreateBody(this.GetDefinition());
      var prev;
      for (var f = body1.m_fixtureList; f;) {
         if (callback(f)) {
            var next = f.m_next;
            if (prev) {
               prev.m_next = next;
            }
            else {
               body1.m_fixtureList = next;
            }
            body1.m_fixtureCount--;
            f.m_next = body2.m_fixtureList;
            body2.m_fixtureList = f;
            body2.m_fixtureCount++;
            f.m_body = body2;
            f = next;
         }
         else {
            prev = f;
            f = f.m_next;
         }
      }
      body1.ResetMassData();
      body2.ResetMassData();
      var center1 = body1.GetWorldCenter();
      var center2 = body2.GetWorldCenter();
      var velocity1 = b2Math.AddVV(linearVelocity, b2Math.CrossFV(angularVelocity, b2Math.SubtractVV(center1, center)));
      var velocity2 = b2Math.AddVV(linearVelocity, b2Math.CrossFV(angularVelocity, b2Math.SubtractVV(center2, center)));
      body1.SetLinearVelocity(velocity1);
      body2.SetLinearVelocity(velocity2);
      body1.SetAngularVelocity(angularVelocity);
      body2.SetAngularVelocity(angularVelocity);
      body1.SynchronizeFixtures();
      body2.SynchronizeFixtures();
      return body2;
   }
   b2Body.prototype.Merge = function (other) {
      var f;
      for (f = other.m_fixtureList;
      f;) {
         var next = f.m_next;
         other.m_fixtureCount--;
         f.m_next = this.m_fixtureList;
         this.m_fixtureList = f;
         this.m_fixtureCount++;
         f.m_body = body2;
         f = next;
      }
      body1.m_fixtureCount = 0;
      var body1 = this;
      var body2 = other;
      var center1 = body1.GetWorldCenter();
      var center2 = body2.GetWorldCenter();
      var velocity1 = body1.GetLinearVelocity().Copy();
      var velocity2 = body2.GetLinearVelocity().Copy();
      var angular1 = body1.GetAngularVelocity();
      var angular = body2.GetAngularVelocity();
      body1.ResetMassData();
      this.SynchronizeFixtures();
   }
   */
    b2Body.prototype.GetMass = function() {
        return this.m_mass;
    }
    b2Body.prototype.GetGravityScale = function() {
        return this.m_gravityScale;
    }
    b2Body.prototype.SetGravityScale = function(scale) {
        this.m_gravityScale = scale;
    }
    b2Body.prototype.GetInertia = function() {
        return this.m_I + this.m_mass * b2Math.DotVV(this.m_sweep.localCenter, this.m_sweep.localCenter);
    }
    b2Body.prototype.GetMassData = function(data) {
        data.mass = this.m_mass;
        data.I = this.m_I + this.m_mass * b2Math.DotVV(this.m_sweep.localCenter, this.m_sweep.localCenter);
        data.center.SetV(this.m_sweep.localCenter);
    }
    b2Body.prototype.SetMassData = function(massData) {
        b2Settings.b2Assert(this.m_world.IsLocked() == false);
        if (this.m_world.IsLocked() == true) {
            return;
        }
        if (this.m_type != b2Body.b2_dynamicBody) {
            return;
        }
        this.m_invMass = 0.0;
        this.m_I = 0.0;
        this.m_invI = 0.0;
        this.m_mass = massData.mass;
        if (this.m_mass <= 0.0) {
            this.m_mass = 1.0;
        }
        this.m_invMass = 1.0 / this.m_mass;
        if (massData.I > 0.0 && (this.m_flags & b2Body.e_fixedRotationFlag) == 0) {
            this.m_I = massData.I - this.m_mass * b2Math.DotVV(massData.center, massData.center);
            this.m_invI = 1.0 / this.m_I;
        }
        var oldCenter = this.m_sweep.c.Copy();
        this.m_sweep.localCenter.SetV(massData.center);
        this.m_sweep.c0.SetV(b2Math.MulXV(this.m_xf, this.m_sweep.localCenter));
        this.m_sweep.c.SetV(this.m_sweep.c0);
        this.m_linearVelocity.Add(b2Math.CrossFV(this.m_angularVelocity, b2Math.SubtractVV(this.m_sweep.c, oldCenter)));
    }
    b2Body.prototype.ResetMassData = function() {
        this.m_mass = 0.0;
        this.m_invMass = 0.0;
        this.m_I = 0.0;
        this.m_invI = 0.0;
        this.m_sweep.localCenter.SetZero();
        if (this.m_type == b2Body.b2_staticBody || this.m_type == b2Body.b2_kinematicBody) {
            this.m_sweep.c0.SetV(this.m_xf.p);
            this.m_sweep.c.SetV(this.m_xf.p);
            this.m_sweep.a0 = this.m_sweep.a;
            return;
        }

        b2Settings.b2Assert(this.m_type === b2Body.b2_dynamicBody);
        var center = new b2Vec2(0, 0);
        for (var f = this.m_fixtureList; f; f = f.m_next) {
            if (f.m_density == 0.0) {
                continue;
            }
            var massData = new b2MassData;
            f.GetMassData(massData);
            this.m_mass += massData.mass;
            center.x += massData.center.x * massData.mass;
            center.y += massData.center.y * massData.mass;
            this.m_I += massData.I;
        }
        if (this.m_mass > 0.0) {
            this.m_invMass = 1.0 / this.m_mass;
            center.x *= this.m_invMass;
            center.y *= this.m_invMass;
        } else {
            this.m_mass = 1.0;
            this.m_invMass = 1.0;
        }
        if (this.m_I > 0.0 && (this.m_flags & b2Body.e_fixedRotationFlag) == 0) {
            this.m_I -= this.m_mass * b2Math.DotVV(center, center);
//            this.m_I *= this.m_gravityScale;
            b2Settings.b2Assert(this.m_I > 0);
            this.m_invI = 1.0 / this.m_I;
        } else {
            this.m_I = 0.0;
            this.m_invI = 0.0;
        }
        var oldCenter = this.m_sweep.c.Copy();
        this.m_sweep.localCenter.SetV(center);
        this.m_sweep.c0.SetV(b2Math.MulXV(this.m_xf, this.m_sweep.localCenter));
        this.m_sweep.c.SetV(this.m_sweep.c0);
        this.m_linearVelocity.Add(b2Math.CrossFV(this.m_angularVelocity, b2Math.SubtractVV(this.m_sweep.c, oldCenter)));
    }
    b2Body.prototype.GetWorldPoint = function(localPoint) {
        return b2Math.MulXV(this.m_xf, localPoint);
    }
    b2Body.prototype.GetWorldVector = function(localVector) {
        return b2Math.MulRV(this.m_xf.q, localVector);
    }
    b2Body.prototype.GetLocalPoint = function(worldPoint) {
        return b2Math.MulTXV(this.m_xf, worldPoint);
    }
    b2Body.prototype.GetLocalVector = function(worldVector) {
        return b2Math.MulTRV(this.m_xf.q, worldVector);
    }
    b2Body.prototype.GetLinearVelocityFromWorldPoint = function(worldPoint) {
        return b2Math.AddVV(this.m_linearVelocity, b2Math.CrossFV(this.m_angularVelocity, b2Math.SubtractVV(worldPoint, this.m_sweep.c)));
    }
    b2Body.prototype.GetLinearVelocityFromLocalPoint = function(localPoint) {
        return this.GetLinearVelocityFromWorldPoint(this.GetWorldPoint(localPoint));
    }
    b2Body.prototype.GetLinearDamping = function() {
        return this.m_linearDamping;
    }
    b2Body.prototype.SetLinearDamping = function(linearDamping) {
        this.m_linearDamping = linearDamping;
    }
    b2Body.prototype.GetAngularDamping = function() {
        return this.m_angularDamping;
    }
    b2Body.prototype.SetAngularDamping = function(angularDamping) {
        this.m_angularDamping = angularDamping;
    }
    b2Body.prototype.SetType = function(type) {
        b2Settings.b2Assert(this.m_world.IsLocked() == false);
        if (this.m_world.IsLocked() == true) {
            return;
        }
        if (this.m_type == type) {
            return;
        }
        this.m_type = type;
        this.ResetMassData();
        if (this.m_type == b2Body.b2_staticBody) {
            this.m_linearVelocity.SetZero();
            this.m_angularVelocity = 0.0;
            this.m_sweep.a0 = this.m_sweep.a;
            this.m_sweep.c0.SetV(this.m_sweep.c);
            SynchronizeFixtures();
        }
        this.SetAwake(true);
        this.m_force.SetZero();
        this.m_torque = 0.0;
        for (var f = this.m_fixtureList; f; f = f.m_next) {
            f.Refilter();
        }
    }
    b2Body.prototype.GetType = function() {
        return this.m_type;
    }
    b2Body.prototype.SetBullet = function(flag) {
        if (flag) {
            this.m_flags |= b2Body.e_bulletFlag;
        } else {
            this.m_flags &= ~b2Body.e_bulletFlag;
        }
    }
    b2Body.prototype.IsBullet = function() {
        return (this.m_flags & b2Body.e_bulletFlag) == b2Body.e_bulletFlag;
    }
    b2Body.prototype.SetSleepingAllowed = function(flag) {
        if (flag) {
            this.m_flags |= b2Body.e_autoSleepFlag;
        } else {
            this.m_flags &= ~b2Body.e_autoSleepFlag;
            this.SetAwake(true);
        }
    }
    b2Body.prototype.SetAwake = function(flag) {
        if (flag) {
            if ((this.m_flags & b2Body.e_awakeFlag) == 0) {
                this.m_flags |= b2Body.e_awakeFlag;
                this.m_sleepTime = 0.0;
            }
        } else {
            this.m_flags &= ~b2Body.e_awakeFlag;
            this.m_sleepTime = 0.0;
            this.m_linearVelocity.SetZero();
            this.m_angularVelocity = 0.0;
            this.m_force.SetZero();
            this.m_torque = 0.0;
        }
    }
    b2Body.prototype.IsAwake = function() {
        return (this.m_flags & b2Body.e_awakeFlag) == b2Body.e_awakeFlag;
    }
    b2Body.prototype.SetFixedRotation = function(fixed) {
        if (fixed) {
            this.m_flags |= b2Body.e_fixedRotationFlag;
        } else {
            this.m_flags &= ~b2Body.e_fixedRotationFlag;
        }
        this.ResetMassData();
    }
    b2Body.prototype.IsFixedRotation = function() {
        return (this.m_flags & b2Body.e_fixedRotationFlag) == b2Body.e_fixedRotationFlag;
    }
    b2Body.prototype.SetActive = function(flag) {
        if (flag == this.IsActive()) {
            return;
        }
        if (flag) {
            this.m_flags |= b2Body.e_activeFlag;
            var broadPhase = this.m_world.m_contactManager.m_broadPhase;
            for (var f = this.m_fixtureList; f; f = f.m_next) {
                f.CreateProxies(broadPhase, this.m_xf);
            }
        } else {
            this.m_flags &= ~b2Body.e_activeFlag;
            var broadPhase = this.m_world.m_contactManager.m_broadPhase;
            for (var f = this.m_fixtureList; f; f = f.m_next) {
                f.DestroyProxies(broadPhase);
            }
            var ce = this.m_contactList;
            while (ce) {
                var ce0 = ce;
                ce = ce.next;
                this.m_world.m_contactManager.Destroy(ce0.contact);
            }
            this.m_contactList = null;
        }
    }
    b2Body.prototype.IsActive = function() {
        return (this.m_flags & b2Body.e_activeFlag) == b2Body.e_activeFlag;
    }
    b2Body.prototype.IsSleepingAllowed = function() {
        return (this.m_flags & b2Body.e_autoSleepFlag) == b2Body.e_autoSleepFlag;
    }
    b2Body.prototype.GetFixtureList = function() {
        return this.m_fixtureList;
    }
    b2Body.prototype.GetJointList = function() {
        return this.m_jointList;
    }
    b2Body.prototype.GetContactList = function() {
        return this.m_contactList;
    }
    b2Body.prototype.GetNext = function() {
        return this.m_next;
    }
    b2Body.prototype.GetUserData = function() {
        return this.m_userData;
    }
    b2Body.prototype.SetUserData = function(data) {
        this.m_userData = data;
    }
    b2Body.prototype.GetWorld = function() {
        return this.m_world;
    }
    b2Body.prototype.b2Body = function(bd, world) {
        b2Settings.b2Assert(bd.position.IsValid());
        b2Settings.b2Assert(bd.linearVelocity.IsValid());
        b2Settings.b2Assert(b2Math.IsValid(bd.angle));
        b2Settings.b2Assert(b2Math.IsValid(bd.angularVelocity));
        b2Settings.b2Assert(b2Math.IsValid(bd.angularDamping) && bd.angularDamping >= 0.0);
        b2Settings.b2Assert(b2Math.IsValid(bd.linearDamping) && bd.linearDamping >= 0.0);

        this.m_flags = 0;
        if (bd.bullet) {
            this.m_flags |= b2Body.e_bulletFlag;
        }
        if (bd.fixedRotation) {
            this.m_flags |= b2Body.e_fixedRotationFlag;
        }
        if (bd.allowSleep) {
            this.m_flags |= b2Body.e_autoSleepFlag;
        }
        if (bd.awake) {
            this.m_flags |= b2Body.e_awakeFlag;
        }
        if (bd.active) {
            this.m_flags |= b2Body.e_activeFlag;
        }
        this.m_world = world;
        this.m_xf.Set(bd.position, bd.angle);
        this.m_sweep.localCenter.SetZero();
        this.m_sweep.c0.SetV(this.m_xf.p);
        this.m_sweep.c.SetV(this.m_xf.p);
        this.m_sweep.a0 = bd.angle;
        this.m_sweep.a = bd.angle;
        this.m_sweep.alpha0 = 0.0;

        this.m_jointList = null;
        this.m_contactList = null;
        this.m_prev = null;
        this.m_next = null;

        this.m_linearVelocity.SetV(bd.linearVelocity);
        this.m_angularVelocity = bd.angularVelocity;
        this.m_linearDamping = bd.linearDamping;
        this.m_angularDamping = bd.angularDamping;
        this.m_gravityScale = bd.gravityScale;

        this.m_force.SetZero();
        this.m_torque = 0.0;
        this.m_sleepTime = 0.0;
        this.m_type = bd.type;
        if (this.m_type == b2Body.b2_dynamicBody) {
            this.m_mass = 1.0;
            this.m_invMass = 1.0;
        } else {
            this.m_mass = 0.0;
            this.m_invMass = 0.0;
        }
        this.m_I = 0.0;
        this.m_invI = 0.0;
 
        this.m_userData = bd.userData;
        this.m_fixtureList = null;
        this.m_fixtureCount = 0;
    }
    b2Body.prototype.SynchronizeFixtures = function () {
        var xf1 = b2Body.s_xf1;
        var tRot = xf1.q;
        tRot.Set(this.m_sweep.a0);
        var tVec = this.m_sweep.localCenter;
        xf1.p.x = this.m_sweep.c0.x - (tRot.c * tVec.x - tRot.s * tVec.y);
        xf1.p.y = this.m_sweep.c0.y - (tRot.s * tVec.x + tRot.c * tVec.y);
        var broadPhase = this.m_world.m_contactManager.m_broadPhase;
        for (var f = this.m_fixtureList; f; f = f.m_next) {
            f.Synchronize(broadPhase, xf1, this.m_xf);
        }
    }
    b2Body.prototype.SynchronizeTransform = function () {
        this.m_xf.q.Set(this.m_sweep.a);
        this.m_xf.p = b2Math.SubtractVV(this.m_sweep.c, b2Math.MulRV(this.m_xf.q, this.m_sweep.localCenter));
    }
    b2Body.prototype.ShouldCollide = function (other) {
        if (this.m_type != b2Body.b2_dynamicBody && other.m_type != b2Body.b2_dynamicBody) {
            return false;
        }
        for (var jn = this.m_jointList; jn; jn = jn.next) {
            if (jn.other == other) if (jn.joint.m_collideConnected == false) {
                return false;
            }
        }
        return true;
    }
    b2Body.prototype.Advance = function (t) {
        this.m_sweep.Advance(t);
        this.m_sweep.c.SetV(this.m_sweep.c0);
        this.m_sweep.a = this.m_sweep.a0;
        this.SynchronizeTransform();
    }
    Box2D.postDefs.push(function() {
        Box2D.Dynamics.b2Body.s_xf1 = new b2Transform();
        Box2D.Dynamics.b2Body.e_islandFlag = 0x0001;
        Box2D.Dynamics.b2Body.e_awakeFlag = 0x0002;
        Box2D.Dynamics.b2Body.e_autoSleepFlag = 0x0004;
        Box2D.Dynamics.b2Body.e_bulletFlag = 0x0008;
        Box2D.Dynamics.b2Body.e_fixedRotationFlag = 0x0010;
        Box2D.Dynamics.b2Body.e_activeFlag = 0x0020;
        Box2D.Dynamics.b2Body.e_toiFlag = 0x0040;
        Box2D.Dynamics.b2Body.b2_staticBody = 0;
        Box2D.Dynamics.b2Body.b2_kinematicBody = 1;
        Box2D.Dynamics.b2Body.b2_dynamicBody = 2;
    });
    b2BodyDef.b2BodyDef = function() {
        this.position = new b2Vec2();
        this.linearVelocity = new b2Vec2();
    };
    b2BodyDef.prototype.b2BodyDef = function() {
        this.userData = null;
        this.position.Set(0.0, 0.0);
        this.angle = 0.0;
        this.linearVelocity.Set(0, 0);
        this.angularVelocity = 0.0;
        this.linearDamping = 0.0;
        this.angularDamping = 0.0;
        this.allowSleep = true;
        this.awake = true;
        this.fixedRotation = false;
        this.bullet = false;
        this.type = b2Body.b2_staticBody;
        this.active = true;
        this.gravityScale = 1.0;
    }
    b2ContactFilter.b2ContactFilter = function() {};
    b2ContactFilter.prototype.ShouldCollide = function(fixtureA, fixtureB) {
        var filter1 = fixtureA.GetFilterData();
        var filter2 = fixtureB.GetFilterData();
        if (filter1.groupIndex == filter2.groupIndex && filter1.groupIndex != 0) {
            return filter1.groupIndex > 0;
        }
        var collide = (filter1.maskBits & filter2.categoryBits) != 0 && (filter1.categoryBits & filter2.maskBits) != 0;
        return collide;
    }
    /*
    b2ContactFilter.prototype.RayCollide = function(userData, fixture) {
        if (!userData) return true;
        return this.ShouldCollide((userData instanceof b2Fixture ? userData : null), fixture);
    }
    */
    Box2D.postDefs.push(function() {
        Box2D.Dynamics.b2ContactFilter.b2_defaultFilter = new b2ContactFilter();
    });
    b2ContactImpulse.b2ContactImpulse = function() {
        this.normalImpulses = new Vector_a2j_Number(b2Settings.b2_maxManifoldPoints);
        this.tangentImpulses = new Vector_a2j_Number(b2Settings.b2_maxManifoldPoints);
        this.count = 0;
    };
    b2ContactListener.b2ContactListener = function() {};
    b2ContactListener.prototype.BeginContact = function(contact) {}
    b2ContactListener.prototype.EndContact = function(contact) {}
    b2ContactListener.prototype.PreSolve = function(contact, oldManifold) {}
    b2ContactListener.prototype.PostSolve = function(contact, impulse) {}
    Box2D.postDefs.push(function() {
        Box2D.Dynamics.b2ContactListener.b2_defaultListener = new b2ContactListener();
    });
    b2ContactManager.b2ContactManager = function() {
        this.m_broadPhase = new b2BroadPhase();
    };
    b2ContactManager.prototype.b2ContactManager = function() {
        this.m_contactList = null;
        this.m_contactCount = 0;
        this.m_contactFilter = b2ContactFilter.b2_defaultFilter;
        this.m_contactListener = b2ContactListener.b2_defaultListener;
        this.m_allocator = null;
    }

    b2ContactManager.prototype.AddPair = function(proxyUserDataA, proxyUserDataB) {
        var proxyA = (proxyUserDataA instanceof b2FixtureProxy ? proxyUserDataA : null);
        var proxyB = (proxyUserDataB instanceof b2FixtureProxy ? proxyUserDataB : null);
        var fixtureA = proxyA.fixture;
        var fixtureB = proxyB.fixture;
        var indexA = proxyA.childIndex;
        var indexB = proxyB.childIndex;
        var bodyA = fixtureA.GetBody();
        var bodyB = fixtureB.GetBody();
        if (bodyA == bodyB) return;
        var edge = bodyB.GetContactList();
        while (edge) {
            if (edge.other == bodyA) {
                var fA = edge.contact.GetFixtureA();
                var fB = edge.contact.GetFixtureB();
                var iA = edge.contact.GetChildIndexA();
                var iB = edge.contact.GetChildIndexB();
                if (fA == fixtureA && fB == fixtureB && iA == indexA && iB == indexB) return;
                if (fA == fixtureB && fB == fixtureA && iA == indexB && iB == indexA) return;
            }
            edge = edge.next;
        }
        if (bodyB.ShouldCollide(bodyA) == false) {
            return;
        }
        if (this.m_contactFilter && this.m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false) {
            return;
        }
        var c = b2Contact.Create(fixtureA, indexA, fixtureB, indexB, this.m_allocator);
        if (c == null) return;
        fixtureA = c.GetFixtureA();
        fixtureB = c.GetFixtureB();
        indexA = c.GetChildIndexA();
        indexB = c.GetChildIndexB();
        bodyA = fixtureA.m_body;
        bodyB = fixtureB.m_body;
        c.m_prev = null;
        c.m_next = this.m_contactList;
        if (this.m_contactList != null) {
            this.m_contactList.m_prev = c;
        }
        this.m_contactList = c;
        c.m_nodeA.contact = c;
        c.m_nodeA.other = bodyB;
        c.m_nodeA.prev = null;
        c.m_nodeA.next = bodyA.m_contactList;
        if (bodyA.m_contactList != null) {
            bodyA.m_contactList.prev = c.m_nodeA;
        }
        bodyA.m_contactList = c.m_nodeA;
        c.m_nodeB.contact = c;
        c.m_nodeB.other = bodyA;
        c.m_nodeB.prev = null;
        c.m_nodeB.next = bodyB.m_contactList;
        if (bodyB.m_contactList != null) {
            bodyB.m_contactList.prev = c.m_nodeB;
        }
        bodyB.m_contactList = c.m_nodeB;
        bodyA.SetAwake(true);
        bodyB.SetAwake(true);
        ++this.m_contactCount;
        return;
    }
    b2ContactManager.prototype.FindNewContacts = function() {
        this.m_broadPhase.UpdatePairs(this);
    }
    b2ContactManager.prototype.Destroy = function(c) {
        var fixtureA = c.GetFixtureA();
        var fixtureB = c.GetFixtureB();
        var bodyA = fixtureA.GetBody();
        var bodyB = fixtureB.GetBody();
        if (this.m_contactListener && c.IsTouching()) {
            this.m_contactListener.EndContact(c);
        }
        if (c.m_prev) {
            c.m_prev.m_next = c.m_next;
        }
        if (c.m_next) {
            c.m_next.m_prev = c.m_prev;
        }
        if (c == this.m_contactList) {
            this.m_contactList = c.m_next;
        }
        if (c.m_nodeA.prev) {
            c.m_nodeA.prev.next = c.m_nodeA.next;
        }
        if (c.m_nodeA.next) {
            c.m_nodeA.next.prev = c.m_nodeA.prev;
        }
        if (c.m_nodeA == bodyA.m_contactList) {
            bodyA.m_contactList = c.m_nodeA.next;
        }
        if (c.m_nodeB.prev) {
            c.m_nodeB.prev.next = c.m_nodeB.next;
        }
        if (c.m_nodeB.next) {
            c.m_nodeB.next.prev = c.m_nodeB.prev;
        }
        if (c.m_nodeB == bodyB.m_contactList) {
            bodyB.m_contactList = c.m_nodeB.next;
        }
        b2Contact.Destroy(c, this.m_allocator);
        --this.m_contactCount;
    }
    b2ContactManager.prototype.Collide = function() {
        var c = this.m_contactList;
        while (c) {
            var fixtureA = c.GetFixtureA();
            var fixtureB = c.GetFixtureB();
            var indexA = c.GetChildIndexA();
            var indexB = c.GetChildIndexB();
            var bodyA = fixtureA.GetBody();
            var bodyB = fixtureB.GetBody();

            if (c.m_flags & b2Contact.e_filterFlag) {
                if (bodyB.ShouldCollide(bodyA) == false) {
                    var cNuke = c;
                    c = cNuke.GetNext();
                    this.Destroy(cNuke);
                    continue;
                }
                if (this.m_contactFilter && this.m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false) {
                    cNuke = c;
                    c = cNuke.GetNext();
                    this.Destroy(cNuke);
                    continue;
                }
                c.m_flags &= ~b2Contact.e_filterFlag;
            }

            var activeA = bodyA.IsAwake() && bodyA.m_type != b2Body.b2_staticBody;
            var activeB = bodyB.IsAwake() && bodyB.m_type != b2Body.b2_staticBody;
            if (activeA == false && activeB == false) {
                c = c.GetNext();
                continue;
            }
            var proxyA = fixtureA.m_proxies[indexA].proxyId;
            var proxyB = fixtureB.m_proxies[indexB].proxyId;
            var overlap = this.m_broadPhase.TestOverlap(proxyA, proxyB);
            if (overlap == false) {
                cNuke = c;
                c = cNuke.GetNext();
                this.Destroy(cNuke);
                continue;
            }
            c.Update(this.m_contactListener);
            c = c.GetNext();
        }
    }
    b2Draw.b2Draw = function() {};
    b2Draw.prototype.b2Draw = function() {}
    b2Draw.prototype.SetFlags = function(flags) {
        if (flags === undefined) flags = 0;
    }
    b2Draw.prototype.GetFlags = function() {}
    b2Draw.prototype.AppendFlags = function(flags) {
        if (flags === undefined) flags = 0;
    }
    b2Draw.prototype.ClearFlags = function(flags) {
        if (flags === undefined) flags = 0;
    }
    b2Draw.prototype.SetSprite = function(sprite) {}
    b2Draw.prototype.GetSprite = function() {}
    b2Draw.prototype.SetDrawScale = function(drawScale) {
        if (drawScale === undefined) drawScale = 0;
    }
    b2Draw.prototype.GetDrawScale = function() {}
    b2Draw.prototype.SetLineThickness = function(lineThickness) {
        if (lineThickness === undefined) lineThickness = 0;
    }
    b2Draw.prototype.GetLineThickness = function() {}
    b2Draw.prototype.SetAlpha = function(alpha) {
        if (alpha === undefined) alpha = 0;
    }
    b2Draw.prototype.GetAlpha = function() {}
    b2Draw.prototype.SetFillAlpha = function(alpha) {
        if (alpha === undefined) alpha = 0;
    }
    b2Draw.prototype.GetFillAlpha = function() {}
    b2Draw.prototype.SetXFormScale = function(xformScale) {
        if (xformScale === undefined) xformScale = 0;
    }
    b2Draw.prototype.GetXFormScale = function() {}
    b2Draw.prototype.DrawPolygon = function(vertices, vertexCount, color) {
        if (vertexCount === undefined) vertexCount = 0;
    }
    b2Draw.prototype.DrawSolidPolygon = function(vertices, vertexCount, color) {
        if (vertexCount === undefined) vertexCount = 0;
    }
    b2Draw.prototype.DrawCircle = function(center, radius, color) {
        if (radius === undefined) radius = 0;
    }
    b2Draw.prototype.DrawSolidCircle = function(center, radius, axis, color) {
        if (radius === undefined) radius = 0;
    }
    b2Draw.prototype.DrawSegment = function(p1, p2, color) {}
    b2Draw.prototype.DrawTransform = function(xf) {}
    Box2D.postDefs.push(function() {
        Box2D.Dynamics.b2Draw.e_shapeBit = 0x0001;
        Box2D.Dynamics.b2Draw.e_jointBit = 0x0002;
        Box2D.Dynamics.b2Draw.e_aabbBit = 0x0004;
        Box2D.Dynamics.b2Draw.e_pairBit = 0x0008;
        Box2D.Dynamics.b2Draw.e_centerOfMassBit = 0x0010;
        Box2D.Dynamics.b2Draw.e_controllerBit = 0x0020;
    });
    b2DestructionListener.b2DestructionListener = function() {};
    b2DestructionListener.prototype.SayGoodbyeJoint = function(joint) {}
    b2DestructionListener.prototype.SayGoodbyeFixture = function(fixture) {}
    b2Filter.b2Filter = function() {
        this.categoryBits = 0x0001;
        this.maskBits = 0xFFFF;
        this.groupIndex = 0;
    };
    b2Filter.prototype.Copy = function() {
        var copy = new b2Filter();
        copy.categoryBits = this.categoryBits;
        copy.maskBits = this.maskBits;
        copy.groupIndex = this.groupIndex;
        return copy;
    }
    b2Fixture.b2Fixture = function() {
        this.m_filter = new b2Filter();
    };
    b2Fixture.prototype.GetType = function() {
        return this.m_shape.GetType();
    }
    b2Fixture.prototype.GetShape = function() {
        return this.m_shape;
    }
    b2Fixture.prototype.SetSensor = function(sensor) {
        if (this.m_isSensor == sensor) return;
        this.m_body.SetAwake(true);
        this.m_isSensor = sensor;
    }
    b2Fixture.prototype.IsSensor = function() {
        return this.m_isSensor;
    }
    b2Fixture.prototype.SetFilterData = function(filter) {
        this.m_filter = filter.Copy();
        this.Refilter();
    }
    b2Fixture.prototype.Refilter = function() {
        if (!this.m_body) return;
        var edge = this.m_body.GetContactList();
        while (edge) {
            var contact = edge.contact;
            var fixtureA = contact.GetFixtureA();
            var fixtureB = contact.GetFixtureB();
            if (fixtureA == this || fixtureB == this) contact.FlagForFiltering();
            edge = edge.next;
        }
        var world = this.m_body.GetWorld();
        if (!world) return;
        var broadPhase = world.m_contactManager.m_broadPhase;
        for (var i = 0; i < this.m_proxyCount; ++i) {
            broadPhase.TouchProxy(this.m_proxies[i].proxyId);
        }
    }
    b2Fixture.prototype.GetFilterData = function() {
        return this.m_filter;
    }
    b2Fixture.prototype.GetBody = function() {
        return this.m_body;
    }
    b2Fixture.prototype.GetNext = function() {
        return this.m_next;
    }
    b2Fixture.prototype.GetUserData = function() {
        return this.m_userData;
    }
    b2Fixture.prototype.SetUserData = function(data) {
        this.m_userData = data;
    }
    b2Fixture.prototype.TestPoint = function(p) {
        return this.m_shape.TestPoint(this.m_body.GetTransform(), p);
    }
    b2Fixture.prototype.RayCast = function(output, input, childIndex) {
        return this.m_shape.RayCast(output, input, this.m_body.GetTransform(), childIndex);
    }
    b2Fixture.prototype.GetMassData = function(massData) {
        this.m_shape.ComputeMass(massData, this.m_density);
    }
    b2Fixture.prototype.SetDensity = function(density) {
        b2Settings.b2Assert(b2Math.IsValid(density) && density >= 0.0);
        this.m_density = density;
    }
    b2Fixture.prototype.GetDensity = function() {
        return this.m_density;
    }
    b2Fixture.prototype.GetFriction = function() {
        return this.m_friction;
    }
    b2Fixture.prototype.SetFriction = function(friction) {
        this.m_friction = friction;
    }
    b2Fixture.prototype.GetRestitution = function() {
        return this.m_restitution;
    }
    b2Fixture.prototype.SetRestitution = function(restitution) {
        this.m_restitution = restitution;
    }
    b2Fixture.prototype.GetAABB = function(childIndex) {
        b2Settings.b2Assert(0 <= childIndex && childIndex < this.m_proxyCount);
        return this.m_proxies[childIndex].aabb;
        //return this.m_aabb;
    }
    b2Fixture.prototype.b2Fixture = function() {
        //this.m_aabb = new b2AABB();
        this.m_proxies = null;
        this.m_userData = null;
        this.m_body = null;
        this.m_next = null;
        this.m_shape = null;
        this.m_density = 0.0;
        this.m_friction = 0.0;
        this.m_restitution = 0.0;
    }
    b2Fixture.prototype.Create = function(allocator, body, def) {
        this.m_userData = def.userData;
        this.m_friction = def.friction;
        this.m_restitution = def.restitution;

        this.m_body = body;
        this.m_next = null;
        this.m_filter = def.filter.Copy();
        this.m_isSensor = def.isSensor;
        this.m_shape = def.shape.Copy();

        // Reserve proxy space
        var childCount = this.m_shape.GetChildCount();
        this.m_proxies = new Vector(childCount);
        for (var i = 0; i < childCount; ++i)
        {
            this.m_proxies[i] = new b2FixtureProxy();
            this.m_proxies[i].fixture = null;
            this.m_proxies[i].proxyId = null;//b2BroadPhase.e_nullProxy;
        }
        this.m_proxyCount = 0;

        this.m_density = def.density;
    }
    b2Fixture.prototype.Destroy = function() {
        // The proxies must be destroyed before calling this.
        b2Settings.b2Assert(this.m_proxyCount == 0);

//        this.m_proxies = null;
        this.m_shape = null;
    }
    b2Fixture.prototype.CreateProxies = function(broadPhase, xf) {
        b2Settings.b2Assert(this.m_proxyCount == 0);

        // Create proxies in the broad-phase.
        this.m_proxyCount = this.m_shape.GetChildCount();

        for (var i = 0; i < this.m_proxyCount; ++i) {
            var proxy = this.m_proxies[i];
            this.m_shape.ComputeAABB(proxy.aabb, xf, i);
            proxy.proxyId = broadPhase.CreateProxy(proxy.aabb, proxy);
            proxy.fixture = this;
            proxy.childIndex = i;
        }
//        this.m_shape.ComputeAABB(this.m_aabb, xf);
//        this.m_proxy = broadPhase.CreateProxy(this.m_aabb, this);
    }
    b2Fixture.prototype.DestroyProxies = function(broadPhase) {
        for (var i = 0; i < this.m_proxyCount; ++i) {
            broadPhase.DestroyProxy(this.m_proxies[i].proxyId);
            this.m_proxies[i].proxyId = null;//b2BroadPhase.e_nullProxy;
        }
        this.m_proxyCount = 0;
    }
    b2Fixture.prototype.Synchronize = function(broadPhase, transform1, transform2) {
        for (var i = 0; i < this.m_proxyCount; ++i) {
            var proxy = this.m_proxies[i];
            var aabb1 = b2Fixture.s_aabb1;
            var aabb2 = b2Fixture.s_aabb2;
            this.m_shape.ComputeAABB(aabb1, transform1, proxy.childIndex);
            this.m_shape.ComputeAABB(aabb2, transform2, proxy.childIndex);
            proxy.aabb.Combine2(aabb1, aabb2);
            var displacement = b2Math.SubtractVV(transform2.p, transform1.p);
            broadPhase.MoveProxy(proxy.proxyId, proxy.aabb, displacement);
        }
    }
    Box2D.postDefs.push(function() {
        Box2D.Dynamics.b2Fixture.s_aabb1 = new b2AABB();
        Box2D.Dynamics.b2Fixture.s_aabb2 = new b2AABB();
    });
    b2FixtureDef.b2FixtureDef = function() {
        this.filter = new b2Filter();
    };
    b2FixtureDef.prototype.b2FixtureDef = function() {
        this.shape = null;
        this.userData = null;
        this.friction = 0.2;
        this.restitution = 0.0;
        this.density = 0.0;
        this.isSensor = false;
    }
    b2FixtureProxy.b2FixtureProxy = function() {
        this.aabb = new b2AABB();
    };
    b2FixtureProxy.prototype.b2FixtureProxy = function() {
        this.fixture = null;
        this.childIndex = 0;
        this.proxyId = null;//b2BroadPhase.e_nullProxy;
    }
    b2Island.b2Island = function() {};
    b2Island.prototype.b2Island = function() {
        this.m_bodies = new Vector();
        this.m_contacts = new Vector();
        this.m_joints = new Vector();
        this.m_velocities = new Vector();
        this.m_positions = new Vector();
    }
    b2Island.prototype.Initialize = function(bodyCapacity, contactCapacity, jointCapacity, allocator, listener, contactSolver) {
        var i = 0;
        this.m_bodyCapacity = bodyCapacity;
        this.m_contactCapacity = contactCapacity;
        this.m_jointCapacity = jointCapacity;
        this.m_bodyCount = 0;
        this.m_contactCount = 0;
        this.m_jointCount = 0;

        this.m_allocator = allocator;
        this.m_listener = listener;
        this.m_contactSolver = contactSolver;

        for (i = this.m_bodies.length; i < bodyCapacity; i++)
            this.m_bodies[i] = null;
        for (i = this.m_contacts.length; i < contactCapacity; i++)
            this.m_contacts[i] = null;
        for (i = this.m_joints.length; i < jointCapacity; i++)
            this.m_joints[i] = null;
        for (i = this.m_velocities.length; i < bodyCapacity; i++)
            this.m_velocities[i] = new b2Velocity();
        for (i = this.m_positions.length; i < bodyCapacity; i++)
            this.m_positions[i] = new b2Position();
    }
    b2Island.prototype.Clear = function() {
        this.m_bodyCount = 0;
        this.m_contactCount = 0;
        this.m_jointCount = 0;
    }
    b2Island.prototype.Solve = function(profile, step, gravity, allowSleep) {
//        var timer = new b2Timer();
        var h = step.dt;
        var i = 0;
        var j = 0;
        var b;
        var joint;
        var c, vX, vY;
        for (i = 0; i < this.m_bodyCount; ++i) {
            b = this.m_bodies[i];
            c = b.m_sweep.c;
            var a = b.m_sweep.a;
            vX = b.m_linearVelocity.x;
            vY = b.m_linearVelocity.y;
            var w = b.m_angularVelocity;

            b.m_sweep.c0.SetV(b.m_sweep.c);
            b.m_sweep.a0 = b.m_sweep.a;

            if (b.m_type == b2Body.b2_dynamicBody) {
                vX += h * (b.m_gravityScale * gravity.x + b.m_invMass * b.m_force.x);
                vY += h * (b.m_gravityScale * gravity.y + b.m_invMass * b.m_force.y);
                w += h * b.m_invI * b.m_torque;
                vX *= b2Math.Clamp(1.0 - h * b.m_linearDamping, 0.0, 1.0);
                vY *= b2Math.Clamp(1.0 - h * b.m_linearDamping, 0.0, 1.0);
                w *= b2Math.Clamp(1.0 - h * b.m_angularDamping, 0.0, 1.0);
            }
            this.m_positions[i].c.SetV(c);
            this.m_positions[i].a = a;
            this.m_velocities[i].v.x = vX;
            this.m_velocities[i].v.y = vY;
            this.m_velocities[i].w = w;
        }
        //timer.Reset();

        var solverData = new b2SolverData();
        solverData.step = step;//.Copy();
        solverData.positions = this.m_positions;
        solverData.velocities = this.m_velocities;

        var contactSolverDef = b2Island.s_b2ContactSolverDef;
        contactSolverDef.step = step;//.Copy();
        contactSolverDef.contacts = this.m_contacts;
        contactSolverDef.count = this.m_contactCount;
        contactSolverDef.positions = this.m_positions;
        contactSolverDef.velocities = this.m_velocities;
        contactSolverDef.allocator = this.m_allocator;

        this.m_contactSolver.Initialize(contactSolverDef);
        var contactSolver = this.m_contactSolver;
        contactSolver.InitializeVelocityConstraints();

        if (step.warmStarting) {
            contactSolver.WarmStart();
        }
        for (i = 0; i < this.m_jointCount; ++i) {
            this.m_joints[i].InitVelocityConstraints(solverData);
        }

//        profile.solveInit = timer.GetMilliseconds();
//        timer.Reset();

        for (i = 0; i < step.velocityIterations; ++i) {
            for (j = 0; j < this.m_jointCount; ++j) {
                this.m_joints[j].SolveVelocityConstraints(solverData);
            }
            contactSolver.SolveVelocityConstraints();
        }

        contactSolver.StoreImpulses();
//        profile.solveVelocity = timer.GetMilliseconds();

        for (i = 0; i < this.m_bodyCount; ++i) {
            c = this.m_positions[i].c;
            var a = this.m_positions[i].a;
            var v = this.m_velocities[i].v;
            var w = this.m_velocities[i].w;

            var translationX = h * v.x;
            var translationY = h * v.y;
            var translationSq = translationX * translationX + translationY * translationY;
            if (translationSq > b2Settings.b2_maxTranslationSquared) {
                var ratio = b2Settings.b2_maxTranslation / Math.sqrt(translationSq);
                v.Multiply(ratio);
            }
            var rotation = h * w;
            if (rotation * rotation > b2Settings.b2_maxRotationSquared) {
                var ratio = b2Settings.b2_maxRotation / b2Math.Abs(rotation);
                w *= ratio;
            }
            translationX = h * v.x;
            translationY = h * v.y;
            c.x += translationX;
            c.y += translationY;
            a += h * w;

//            this.m_positions[i].c = c;
            this.m_positions[i].a = a;
//            this.m_velocities[i].v = v;
            this.m_velocities[i].w = w;
        }

        //timer.Reset();
        var positionSolved = false;
        for (i = 0; i < step.positionIterations; ++i) {
            var contactsOkay = contactSolver.SolvePositionConstraints();

            var jointsOkay = true;
            for (j = 0; j < this.m_jointCount; ++j) {
                var jointOkay = this.m_joints[j].SolvePositionConstraints(solverData);
                jointsOkay = jointsOkay && jointOkay;
            }
            if (contactsOkay && jointsOkay) {
            // Exit early if the position errors are small.
                positionSolved = true;
                break;
            }
        }

        // Copy state buffers back to the bodies
        for (i = 0; i < this.m_bodyCount; ++i) {
            b = this.m_bodies[i];
            b.m_sweep.c.SetV(this.m_positions[i].c);
            b.m_sweep.a = this.m_positions[i].a;
            b.m_linearVelocity.SetV(this.m_velocities[i].v);
            b.m_angularVelocity = this.m_velocities[i].w;
            b.SynchronizeTransform();
        }

//        profile.solvePosition = timer.GetMilliseconds();

        this.Report(contactSolver.m_velocityConstraints);

        if (allowSleep) {
            var minSleepTime = b2Settings.b2_maxFloat;

            var linTolSqr = b2Settings.b2_linearSleepTolerance * b2Settings.b2_linearSleepTolerance;
            var angTolSqr = b2Settings.b2_angularSleepTolerance * b2Settings.b2_angularSleepTolerance;

            for (i = 0; i < this.m_bodyCount; ++i) {
                b = this.m_bodies[i];
                if (b.GetType() == b2Settings.b2_staticBody) {
                    continue;
                }

                if ((b.m_flags & b2Body.e_autoSleepFlag) == 0 ||
                    b.m_angularVelocity * b.m_angularVelocity > angTolSqr ||
                    b2Math.DotVV(b.m_linearVelocity, b.m_linearVelocity) > linTolSqr) {
                    b.m_sleepTime = 0.0;
                    minSleepTime = 0.0;
                } else {
                    b.m_sleepTime += h;
                    minSleepTime = b2Math.Min(minSleepTime, b.m_sleepTime);
                }
            }

            if (minSleepTime >= b2Settings.b2_timeToSleep && positionSolved) {
                for (var i = 0; i < this.m_bodyCount; ++i) {
                    b = this.m_bodies[i];
                    b.SetAwake(false);
                }
            }
        }
    }
    b2Island.prototype.SolveTOI = function(subStep, toiIndexA, toiIndexB) {
        b2Settings.b2Assert(toiIndexA < this.m_bodyCount);
        b2Settings.b2Assert(toiIndexB < this.m_bodyCount);

        // Initialize the body state.
        for (var i = 0; i < this.m_bodyCount; ++i) {
            var b = this.m_bodies[i];
            this.m_positions[i].c.SetV(b.m_sweep.c);
            this.m_positions[i].a = b.m_sweep.a;
            this.m_velocities[i].v.SetV(b.m_linearVelocity);
            this.m_velocities[i].w = b.m_angularVelocity;
        }

        var contactSolverDef = new b2ContactSolverDef();
        contactSolverDef.contacts = this.m_contacts;
        contactSolverDef.count = this.m_contactCount;
        contactSolverDef.allocator = this.m_allocator;
        contactSolverDef.step = subStep;//.Copy();
        contactSolverDef.positions = this.m_positions;
        contactSolverDef.velocities = this.m_velocites;

        var contactSolver = this.m_b2ContactSolver;
        contactSolver.Initialize(contactSolverDef);

        // Solve position constraints.
        for (var i = 0; i < subStep.positionIterations; ++i) {
            var contactsOkay = contactSolver.SolveTOIPositionConstraints(toiIndexA, toiIndexB);
            if (contactsOkay) {
                break;
            }
        }

        // Leap of faith to new safe state.
        this.m_bodies[toiIndexA].m_sweep.c0.SetV(this.m_positions[toiIndexA].c);
        this.m_bodies[toiIndexA].m_sweep.a0 = this.m_positions[toiIndexA].a;
        this.m_bodies[toiIndexB].m_sweep.c0.SetV(this.m_positions[toiIndexB].c);
        this.m_bodies[toiIndexB].m_sweep.a0 = this.m_positions[toiIndexB].a;

        // No warm starting is needed for TOI events because warm
        // starting impulses were applied in the discrete solver.
        contactSolver.InitializeVelocityConstraints();

        // Solve velocity constraints.
        for (var i = 0; i < subStep.velocityIterations; ++i) {
            contactSolver.SolveVelocityConstraints();
        }

        // Don't store the TOI contact forces for warm starting
        // because they can be quite large.
        var h = subStep.dt;

        // Integrate positions
        for (var i = 0; i < this.m_bodyCount; ++i) {
            var c = this.m_positions[i].c;
            var a = this.m_positions[i].a;
            var v = this.m_velocities[i].v;
            var w = this.m_velocities[i].w;

            // Check for large velocities
            var translation = b2Math.MulFV(h, v);
            if (b2Math.DotVV(translation, translation) > b2Settings.b2_maxTranslationSquared) {
                var ratio = b2Settings.b2_maxTranslation / translation.Length();
                v.Multiply(ratio);
            }

            var rotation = h * w;
            if (rotation * rotation > b2Settings.b2_maxRotationSquared) {
                var ratio = b2Settings.b2_maxRotation / b2Math.Abs(rotation);
                w *= ratio;
            }

            // Integrate
            c.Add(b2Math.MulFV(h, v));
            a += h * w;

//            this.m_positions[i].c = c.Copy();
            this.m_positions[i].a = a;
//            this.m_velocities[i].v = v.Copy();
            this.m_velocities[i].w = w;

            // Sync bodies
            var body = this.m_bodies[i];
            body.m_sweep.c.SetV(c);
            body.m_sweep.a = a;
            body.m_linearVelocity.SetV(v);
            body.m_angularVelocity = w;
            body.SynchronizeTransform();
        }

        Report(contactSolver.m_velocityConstraints);
    }
    b2Island.prototype.Report = function(constraints) {
        if (this.m_listener == null) {
            return;
        }
        for (var i = 0; i < this.m_contactCount; ++i) {
            var c = this.m_contacts[i];
            var vc = constraints[i];
            b2Island.s_impulse.count = vc.pointCount;
            for (var j = 0; j < vc.pointCount; ++j) {
                b2Island.s_impulse.normalImpulses[j] = vc.points[j].normalImpulse;
                b2Island.s_impulse.tangentImpulses[j] = vc.points[j].tangentImpulse;
            }
            this.m_listener.PostSolve(c, b2Island.s_impulse);
        }
    }
    b2Island.prototype.AddBody = function(body) {
//        b2Settings.b2Assert(this.m_bodyCount < this.m_bodyCapacity);
        body.m_islandIndex = this.m_bodyCount;
        this.m_bodies[this.m_bodyCount++] = body;
    }
    b2Island.prototype.AddContact = function(contact) {
//        b2Settings.b2Assert(this.m_contactCount < this.m_contactCapacity);
        this.m_contacts[this.m_contactCount++] = contact;
    }
    b2Island.prototype.AddJoint = function(joint) {
//        b2Settings.b2Assert(this.m_jointCount < this.m_jointCapacity);
        this.m_joints[this.m_jointCount++] = joint;
    }
    Box2D.postDefs.push(function() {
        Box2D.Dynamics.b2Island.s_impulse = new b2ContactImpulse();
        Box2D.Dynamics.b2Island.s_b2ContactSolverDef = new b2ContactSolverDef();
    });
    b2Position.b2Position = function() {
        this.c = new b2Vec2();
        this.a = 0;
    };
    b2Profile.b2Profile = function() {};
    b2Profile.prototype.Reset = function() {
        this.step = 0;
        this.collide = 0;
        this.solve = 0;
        this.solveInit = 0;
        this.solveVelocity = 0;
        this.solvePosition = 0;
        this.broadphase = 0;
        this.solveTOI = 0;
    }
    b2SolverData.b2SolverData = function() {
        this.step = null;//new b2TimeStep();
        this.positions = null;
        this.velocities = null;
    };
    b2TimeStep.b2TimeStep = function() {};
    b2TimeStep.prototype.Copy = function() {
        var ret = new b2TimeStep();
        ret.Set(this);
        return ret;
    }
    b2TimeStep.prototype.Set = function(step) {
        this.dt = step.dt;
        this.inv_dt = step.inv_dt;
        this.dtRatio = step.dtRatio;
        this.positionIterations = step.positionIterations;
        this.velocityIterations = step.velocityIterations;
        this.warmStarting = step.warmStarting;
    }
    b2Velocity.b2Velocity = function() {
        this.v = new b2Vec2();
        this.w = 0.0;
    };
    b2World.b2World = function() {
        this.s_stack = new Vector();
        this.m_contactManager = new b2ContactManager();
        this.m_contactSolver = new b2ContactSolver();
        this.m_island = new b2Island();
        this.m_gravity = new b2Vec2();
        //this.m_profile = new b2Profile();
    };
    b2World.prototype.b2World = function(gravity) {
        this.m_destructionListener = null;
        this.m_debugDraw = null;

        this.m_bodyList = null;
        this.m_jointList = null;

        this.m_bodyCount = 0;
        this.m_jointCount = 0;

        this.m_warmStarting = true;
        this.m_continuousPhysics = true;
        this.m_subStepping = false;

        this.m_stepComplete = true;

        this.m_allowSleep = true;
        this.m_gravity.SetV(gravity);

        this.m_flags = b2World.e_clearForces;

        this.m_inv_dt0 = 0.0;

        this.m_contactManager.m_allocator = this.m_blockAllocator;

//        this.m_profile.Reset();
    }
    b2World.prototype.SetDestructionListener = function(listener) {
        this.m_destructionListener = listener;
    }
    b2World.prototype.SetContactFilter = function(filter) {
        this.m_contactManager.m_contactFilter = filter;
    }
    b2World.prototype.SetContactListener = function(listener) {
        this.m_contactManager.m_contactListener = listener;
    }
    b2World.prototype.SetDebugDraw = function(debugDraw) {
        this.m_debugDraw = debugDraw;
    }
    /*
    b2World.prototype.SetBroadPhase = function(broadPhase) {
        var oldBroadPhase = this.m_contactManager.m_broadPhase;
        this.m_contactManager.m_broadPhase = broadPhase;
        for (var b = this.m_bodyList; b; b = b.m_next) {
            for (var f = b.m_fixtureList; f; f = f.m_next) {
                f.m_proxy = broadPhase.CreateProxy(oldBroadPhase.GetFatAABB(f.m_proxy), f);
            }
        }
    }
    */
    b2World.prototype.Validate = function() {
        this.m_contactManager.m_broadPhase.Validate();
    }
    b2World.prototype.GetProxyCount = function() {
        return this.m_contactManager.m_broadPhase.GetProxyCount();
    }
    b2World.prototype.GetTreeHeight = function() {
        return this.m_contactManager.m_broadPhase.GetTreeHeight();
    }
    b2World.prototype.GetTreeBalance = function() {
        return this.m_contactManager.m_broadPhase.GetTreeBalance();
    }
    b2World.prototype.GetTreeQuality = function() {
        return this.m_contactManager.m_broadPhase.GetTreeQuality();
    }
    b2World.prototype.CreateBody = function(def) {
        if (this.IsLocked() == true) {
            return null;
        }
        var b = new b2Body(def, this);
        b.m_prev = null;
        b.m_next = this.m_bodyList;
        if (this.m_bodyList) {
            this.m_bodyList.m_prev = b;
        }
        this.m_bodyList = b;
        ++this.m_bodyCount;
        return b;
    }
    b2World.prototype.DestroyBody = function(b) {
        b2Settings.b2Assert(this.m_bodyCount > 0);
        if (this.IsLocked() == true) {
            return;
        }
        var jn = b.m_jointList;
        while (jn) {
            var jn0 = jn;
            jn = jn.next;
            if (this.m_destructionListener) {
                this.m_destructionListener.SayGoodbyeJoint(jn0.joint);
            }
            this.DestroyJoint(jn0.joint);
            b.m_jointList = jn;
        }
        b.m_jointList = null;

        var ce = b.m_contactList;
        while (ce) {
            var ce0 = ce;
            ce = ce.next;
            this.m_contactManager.Destroy(ce0.contact);
        }
        b.m_contactList = null;

        var f = b.m_fixtureList;
        while (f) {
            var f0 = f;
            f = f.m_next;
            if (this.m_destructionListener) {
                this.m_destructionListener.SayGoodbyeFixture(f0);
            }
            f0.DestroyProxies(this.m_contactManager.m_broadPhase);
            f0.Destroy(this.m_blockAllocator);
            f0 = null;
            b.m_fixtureList = f;
            b.m_fixtureCount -= 1;
        }
        b.m_fixtureList = null;
        b.m_fixtureCount = 0;

        if (b.m_prev) {
            b.m_prev.m_next = b.m_next;
        }
        if (b.m_next) {
            b.m_next.m_prev = b.m_prev;
        }
        if (b == this.m_bodyList) {
            this.m_bodyList = b.m_next;
        }
        --this.m_bodyCount;
        b = null;
    }
    b2World.prototype.CreateJoint = function(def) {
        if (this.IsLocked()) {
            return null;
        }
        var j = b2Joint.Create(def, null);
        j.m_prev = null;
        j.m_next = this.m_jointList;
        if (this.m_jointList) {
            this.m_jointList.m_prev = j;
        }
        this.m_jointList = j;
        ++this.m_jointCount;

        j.m_edgeA.joint = j;
        j.m_edgeA.other = j.m_bodyB;
        j.m_edgeA.prev = null;
        j.m_edgeA.next = j.m_bodyA.m_jointList;
        if (j.m_bodyA.m_jointList) j.m_bodyA.m_jointList.prev = j.m_edgeA;
        j.m_bodyA.m_jointList = j.m_edgeA;

        j.m_edgeB.joint = j;
        j.m_edgeB.other = j.m_bodyA;
        j.m_edgeB.prev = null;
        j.m_edgeB.next = j.m_bodyB.m_jointList;
        if (j.m_bodyB.m_jointList) j.m_bodyB.m_jointList.prev = j.m_edgeB;
        j.m_bodyB.m_jointList = j.m_edgeB;

        var bodyA = def.bodyA;
        var bodyB = def.bodyB;
        if (def.collideConnected == false) {
            var edge = bodyB.GetContactList();
            while (edge) {
                if (edge.other == bodyA) {
                    edge.contact.FlagForFiltering();
                }
                edge = edge.next;
            }
        }
        return j;
    }
    b2World.prototype.DestroyJoint = function(j) {
        if (this.IsLocked()) {
            return;
        }
        var collideConnected = j.m_collideConnected;
        if (j.m_prev) {
            j.m_prev.m_next = j.m_next;
        }
        if (j.m_next) {
            j.m_next.m_prev = j.m_prev;
        }
        if (j == this.m_jointList) {
            this.m_jointList = j.m_next;
        }
        var bodyA = j.m_bodyA;
        var bodyB = j.m_bodyB;
        bodyA.SetAwake(true);
        bodyB.SetAwake(true);
        if (j.m_edgeA.prev) {
            j.m_edgeA.prev.next = j.m_edgeA.next;
        }
        if (j.m_edgeA.next) {
            j.m_edgeA.next.prev = j.m_edgeA.prev;
        }
        if (j.m_edgeA == bodyA.m_jointList) {
            bodyA.m_jointList = j.m_edgeA.next;
        }
        j.m_edgeA.prev = null;
        j.m_edgeA.next = null;
        if (j.m_edgeB.prev) {
            j.m_edgeB.prev.next = j.m_edgeB.next;
        }
        if (j.m_edgeB.next) {
            j.m_edgeB.next.prev = j.m_edgeB.prev;
        }
        if (j.m_edgeB == bodyB.m_jointList) {
            bodyB.m_jointList = j.m_edgeB.next;
        }
        j.m_edgeB.prev = null;
        j.m_edgeB.next = null;

        b2Joint.Destroy(j, null);
        --this.m_jointCount;

        if (collideConnected == false) {
            var edge = bodyB.GetContactList();
            while (edge) {
                if (edge.other == bodyA) {
                    edge.contact.FlagForFiltering();
                }
                edge = edge.next;
            }
        }
    }
    b2World.prototype.SetWarmStarting = function(flag) {
        this.m_warmStarting = flag;
    }
    b2World.prototype.GetWarmStarting = function() {
        return this.m_warmStarting;
    }
    b2World.prototype.SetContinuousPhysics = function(flag) {
        this.m_continuousPhysics = flag;
    }
    b2World.prototype.GetContinuousPhysics = function() {
        return this.m_continuousPhysics;
    }
    b2World.prototype.SetAutoClearForces = function(flag) {
        if (flag) {
            this.m_flags |= b2World.e_clearForces;
        } else {
            this.m_flags &= ~b2World.e_clearForces;
        }
    }
    b2World.prototype.GetAutoClearForces = function() {
        return (this.m_flags & b2World.e_clearForces) === b2World.e_clearForces;
    }
    b2World.prototype.SetAllowSleeping = function(flag) {
        if (flag == this.m_allowSleep) {
            return;
        }
        this.m_allowSleep = flag;
        if (this.m_allowSleep == false) {
            for (var b = this.m_bodyList; b; b = b.next) {
                b.SetAwake(true);
            }
        }
    }
    b2World.prototype.GetBodyCount = function() {
        return this.m_bodyCount;
    }
    b2World.prototype.GetJointCount = function() {
        return this.m_jointCount;
    }
    b2World.prototype.GetContactManager = function() {
        return this.m_contactManager;
    }
    b2World.prototype.GetContactCount = function() {
        return this.m_contactManager.m_contactCount;
    }
    b2World.prototype.SetGravity = function(gravity) {
        this.m_gravity = gravity;
    }
    b2World.prototype.GetGravity = function() {
        return this.m_gravity;
    }
    b2World.prototype.GetGroundBody = function() {
        return this.m_groundBody;
    }
    b2World.prototype.Step = function(dt, velocityIterations, positionIterations) {
//        var stepTimer = new b2Timer();
        if (this.m_flags & b2World.e_newFixture) {
            this.m_contactManager.FindNewContacts();
            this.m_flags &= ~b2World.e_newFixture;
        }
        this.m_flags |= b2World.e_locked;
        var step = b2World.s_timestep2;
        step.dt = dt;
        step.velocityIterations = velocityIterations;
        step.positionIterations = positionIterations;
        if (dt > 0.0) {
            step.inv_dt = 1.0 / dt;
        } else {
            step.inv_dt = 0.0;
        }
        step.dtRatio = this.m_inv_dt0 * dt;
        step.warmStarting = this.m_warmStarting;

//        var timer = new b2Timer();
        this.m_contactManager.Collide();
//        this.m_profile.collide = timer.GetMilliseconds();

        if (this.m_stepComplete && step.dt > 0.0) {
            //var timer = new b2Timer();
            this.Solve(step);
//        this.m_profile.solve = timer.GetMilliseconds();
        }
        if (this.m_continuousPhysics && step.dt > 0.0) {
            //var timer = new b2Timer();
            this.SolveTOI(step);
            //this.m_profile.solveTOI = timer.GetMilliseconds();
        }
        if (step.dt > 0.0) {
            this.m_inv_dt0 = step.inv_dt;
        }

        if (this.m_flags & b2World.e_clearForces)
        {
            this.ClearForces();
        }

        this.m_flags &= ~b2World.e_locked;
//        this.m_profile.step = stepTimer.GetMilliseconds();
    }
    b2World.prototype.ClearForces = function() {
        for (var body = this.m_bodyList; body; body = body.m_next) {
            body.m_force.SetZero();
            body.m_torque = 0.0;
        }
    }
    b2World.prototype.DrawDebugData = function() {
        if (this.m_debugDraw == null) {
            return;
        }
        this.m_debugDraw.m_sprite.graphics.clear();
        var flags = this.m_debugDraw.GetFlags();
        var i = 0;
        var b;
        var f;
        var s;
        var j;
        var bp;
        var invQ = new b2Vec2;
        var x1 = new b2Vec2;
        var x2 = new b2Vec2;
        var xf;
        var b1 = new b2AABB();
        var b2 = new b2AABB();
        var vs = [new b2Vec2(), new b2Vec2(), new b2Vec2(), new b2Vec2()];
        var color = new b2Color(0, 0, 0);
        if (flags & b2Draw.e_shapeBit) {
            for (b = this.m_bodyList;
            b; b = b.m_next) {
                xf = b.m_xf;
                for (f = b.GetFixtureList();
                f; f = f.m_next) {
                    s = f.GetShape();
                    if (b.IsActive() == false) {
                        color.Set(0.5, 0.5, 0.3);
                        this.DrawShape(s, xf, color);
                    } else if (b.GetType() == b2Body.b2_staticBody) {
                        color.Set(0.5, 0.9, 0.5);
                        this.DrawShape(s, xf, color);
                    } else if (b.GetType() == b2Body.b2_kinematicBody) {
                        color.Set(0.5, 0.5, 0.9);
                        this.DrawShape(s, xf, color);
                    } else if (b.IsAwake() == false) {
                        color.Set(0.6, 0.6, 0.6);
                        this.DrawShape(s, xf, color);
                    } else {
                        color.Set(0.9, 0.7, 0.7);
                        this.DrawShape(s, xf, color);
                    }
                }
            }
        }
        if (flags & b2Draw.e_jointBit) {
            for (j = this.m_jointList;
            j; j = j.m_next) {
                this.DrawJoint(j);
            }
        }
        if (flags & b2Draw.e_controllerBit) {
            for (var c = this.m_controllerList; c; c = c.m_next) {
                c.Draw(this.m_debugDraw);
            }
        }
        if (flags & b2Draw.e_pairBit) {
            color.Set(0.3, 0.9, 0.9);
            for (var contact = this.m_contactManager.m_contactList; contact; contact = contact.GetNext()) {
                var fixtureA = contact.GetFixtureA();
                var fixtureB = contact.GetFixtureB();
                var cA = fixtureA.GetAABB().GetCenter();
                var cB = fixtureB.GetAABB().GetCenter();
                this.m_debugDraw.DrawSegment(cA, cB, color);
            }
        }
        if (flags & b2Draw.e_aabbBit) {
            bp = this.m_contactManager.m_broadPhase;
            vs = [new b2Vec2(), new b2Vec2(), new b2Vec2(), new b2Vec2()];
            for (b = this.m_bodyList;
            b; b = b.GetNext()) {
                if (b.IsActive() == false) {
                    continue;
                }
                for (f = b.GetFixtureList();
                f; f = f.GetNext()) {
                    var aabb = bp.GetFatAABB(f.m_proxy);
                    vs[0].Set(aabb.lowerBound.x, aabb.lowerBound.y);
                    vs[1].Set(aabb.upperBound.x, aabb.lowerBound.y);
                    vs[2].Set(aabb.upperBound.x, aabb.upperBound.y);
                    vs[3].Set(aabb.lowerBound.x, aabb.upperBound.y);
                    this.m_debugDraw.DrawPolygon(vs, 4, color);
                }
            }
        }
        if (flags & b2Draw.e_centerOfMassBit) {
            for (b = this.m_bodyList;
            b; b = b.m_next) {
                xf = b2World.s_xf;
                xf.q = b.m_xf.q;
                xf.p = b.GetWorldCenter();
                this.m_debugDraw.DrawTransform(xf);
            }
        }
    }
    b2WorldQueryWrapper.b2WorldQueryWrapper = function() {}
    b2WorldQueryWrapper.prototype.QueryCallback = function(proxyId) {
        var proxy = this.broadPhase.GetUserData(proxyId);
        return this.callback.ReportFixture(proxy.fixture);
    };
    b2World.prototype.QueryAABB = function(callback, aabb) {
        var __this = this;
        var wrapper = new b2WorldQueryWrapper();
        wrapper.broadPhase = __this.m_contactManager.m_broadPhase;
        wrapper.callback = callback;
        this.m_contactManager.m_broadPhase.Query(wrapper, aabb);
    }
    b2World.prototype.QueryShape = function(callback, shape, transform) {
        var __this = this;
        if (transform === undefined) transform = null;
        if (transform == null) {
            transform = new b2Transform();
            transform.SetIdentity();
        }
        var broadPhase = __this.m_contactManager.m_broadPhase;

        function WorldQueryWrapper(proxy) {
            var fixture = (broadPhase.GetUserData(proxy) instanceof b2Fixture ? broadPhase.GetUserData(proxy) : null);
            if (b2Shape.TestOverlap(shape, transform, fixture.GetShape(), fixture.GetBody().GetTransform())) return callback(fixture);
            return true;
        };
        var aabb = new b2AABB();
        shape.ComputeAABB(aabb, transform);
        broadPhase.Query(WorldQueryWrapper, aabb);
    }
    b2World.prototype.QueryPoint = function(callback, p) {
        var __this = this;
        var broadPhase = __this.m_contactManager.m_broadPhase;

        function WorldQueryWrapper(proxy) {
            var fixture = (broadPhase.GetUserData(proxy) instanceof b2Fixture ? broadPhase.GetUserData(proxy) : null);
            if (fixture.TestPoint(p)) return callback(fixture);
            return true;
        };
        var aabb = new b2AABB();
        aabb.lowerBound.Set(p.x - b2Settings.b2_linearSlop, p.y - b2Settings.b2_linearSlop);
        aabb.upperBound.Set(p.x + b2Settings.b2_linearSlop, p.y + b2Settings.b2_linearSlop);
        broadPhase.Query(WorldQueryWrapper, aabb);
    };
    b2WorldRayCastWrapper.b2WorldRayCastWrapper = function() {};
    b2WorldRayCastWrapper.prototype.RayCastCallback = function(input, proxyId) {
        var proxy = broadPhase.GetUserData(proxyId);
        var fixture = proxy.fixture;
        var index = proxy.childIndex;
        var output = new b2RayCastOutput;
        var hit = fixture.RayCast(output, input, index);
        if (hit) {
            var fraction = output.fraction;
            var point = b2Math.AddVV(b2Math.MulFV(1.0 - fraction, input.p1), b2Math.MulFV(fraction, input.p2));
            return callback.ReportFixture(fixture, point, output.normal, fraction);
        }
        return input.maxFraction;
    };
    b2World.prototype.RayCast = function(callback, point1, point2) {
        var wrapper = new b2WorldRayCastWrapper();
        wrapper.broadPhase = this.m_contactManager.m_broadPhase;
        wrapper.callback = callback;
        var input = new b2RayCastInput();
        input.maxFraction = 1.0;
        input.p1.SetV(point1);
        input.p2.SetV(point2);
        this.m_contactManager.m_broadPhase.RayCast(wrapper, input);
    }
    b2World.prototype.GetBodyList = function() {
        return this.m_bodyList;
    }
    b2World.prototype.GetJointList = function() {
        return this.m_jointList;
    }
    b2World.prototype.GetContactList = function() {
        return this.m_contactManager.m_contactList;
    }
    b2World.prototype.IsLocked = function() {
        return (this.m_flags & b2World.e_locked) == b2World.e_locked;
    }
    b2World.prototype.Solve = function(step) {
        //this.m_profile.solveInit = 0.0;
        //this.m_profile.solveVelocity = 0.0;
        //this.m_profile.solvePosition = 0.0;

        var b;
        var island = this.m_island;
        island.Initialize(this.m_bodyCount, this.m_contactManager.m_contactCount, this.m_jointCount, null, this.m_contactManager.m_contactListener, this.m_contactSolver);

        for (b = this.m_bodyList; b; b = b.m_next) {
            b.m_flags &= ~b2Body.e_islandFlag;
        }
        for (var c = this.m_contactManager.m_contactList; c; c = c.m_next) {
            c.m_flags &= ~b2Contact.e_islandFlag;
        }
        for (var j = this.m_jointList; j; j = j.m_next) {
            j.m_islandFlag = false;
        }
        var stackSize = parseInt(this.m_bodyCount);
        var stack = this.s_stack;
        for (var i = 0; i < stackSize; ++i) {
            stack[i] = null;
        }
        for (var seed = this.m_bodyList; seed; seed = seed.m_next) {
            if (seed.m_flags & b2Body.e_islandFlag) {
                continue;
            }
            if (seed.IsAwake() == false || seed.IsActive() == false) {
                continue;
            }
            if (seed.GetType() == b2Body.b2_staticBody) {
                continue;
            }
            island.Clear();
            var stackCount = 0;
            stack[stackCount++] = seed;
            seed.m_flags |= b2Body.e_islandFlag;

            while (stackCount > 0) {
                b = stack[--stackCount];
                //b2Settings.b2Assert(b.IsActive());
                island.AddBody(b);

                b.SetAwake(true);

                if (b.GetType() == b2Body.b2_staticBody) {
                    continue;
                }
                var other;
                for (var ce = b.m_contactList; ce; ce = ce.next) {
                    if (ce.contact.m_flags & b2Contact.e_islandFlag) {
                        continue;
                    }
                    if (ce.contact.IsEnabled() == false || ce.contact.IsTouching() == false) {
                        continue;
                    }
                    var sensorA = ce.contact.m_fixtureA.IsSensor();
                    var sensorB = ce.contact.m_fixtureB.IsSensor();
                    if (sensorA || sensorB) {
                        continue;
                    }

                    island.AddContact(ce.contact);
                    ce.contact.m_flags |= b2Contact.e_islandFlag;
                    other = ce.other;
                    if (other.m_flags & b2Body.e_islandFlag) {
                        continue;
                    }
                    //b2Settings.b2Assert(stackCount < stackSize);
                    stack[stackCount++] = other;
                    other.m_flags |= b2Body.e_islandFlag;
                }
                for (var jn = b.m_jointList; jn; jn = jn.next) {
                    if (jn.joint.m_islandFlag == true) {
                        continue;
                    }
                    other = jn.other;
                    if (other.IsActive() == false) {
                        continue;
                    }
                    island.AddJoint(jn.joint);
                    jn.joint.m_islandFlag = true;
                    if (other.m_flags & b2Body.e_islandFlag) {
                        continue;
                    }
                    //b2Settings.b2Assert(stackCount < stackSize);
                    stack[stackCount++] = other;
                    other.m_flags |= b2Body.e_islandFlag;
                }
            }

            var profile;// = new b2Profile();
            island.Solve(profile, step, this.m_gravity, this.m_allowSleep);
            //this.m_profile.solveInit += profile.solveInit;
            //this.m_profile.solveVelocity += profile.solveVelocity;
            //this.m_profile.solvePosition += profile.solvePosition;

            for (var i = 0; i < island.m_bodyCount; ++i) {
                b = island.m_bodies[i];
                if (b.GetType() == b2Body.b2_staticBody) {
                    b.m_flags &= ~b2Body.e_islandFlag;
                }
            }
        }

        for (var i = 0; i < stack.length; ++i) {
            if (!stack[i]) break;
            stack[i] = null;
        }

//        var timer = new b2Timer();

        for (b = this.m_bodyList; b; b = b.m_next) {
            if ((b.m_flags & b2Body.e_islandFlag) == 0) {
                continue;
            }
            if (b.GetType() == b2Body.b2_staticBody) {
                continue;
            }
            b.SynchronizeFixtures();
        }
        this.m_contactManager.FindNewContacts();
        //this.m_profile.broadphase = timer.GetMilliseconds();
        //timer = null;
    }
    b2World.prototype.SolveTOI = function(step) {
        var island = this.m_island;
        island.Initialize(2 * b2Settings.b2_maxTOIContacts, b2Settings.b2_maxTOIContacts, 0, this.m_stackAllocator, this.m_contactManager.m_contactListener, this.m_contactSolver);

        var b;
        if (this.m_stepComplete) {
            for (b = this.m_bodyList; b; b = b.m_next) {
                b.m_flags &= ~b2Body.e_islandFlag;
                b.m_sweep.alpha0 = 0.0;
            }
            for (var c = this.m_contactManager.m_contactList; c; c = c.m_next) {
                c.m_flags &= ~(b2Contact.e_toiFlag | b2Contact.e_islandFlag);
                c.m_toiCount = 0;
                c.m_toi = 1.0;
            }
        }
        for (;;) {
            var minContact = null;
            var minAlpha = 1.0;
            for (var c = this.m_contactManager.m_contactList; c; c = c.m_next) {
                if (c.IsEnabled() == false) {
                    continue;
                }
                if (c.m_toiCount > b2Settings.b2_maxSubSteps) {
                    continue;
                }
                var alpha = 1.0;
                if (c.m_flags & b2Contact.e_toiFlag) {
                    alpha = c.m_toi;
                } else {
                    var fA = c.GetFixtureA();
                    var fB = c.GetFixtureB();
                    if (fA.IsSensor() || fB.IsSensor()) {
                        continue;
                    }
                    var bA = fA.GetBody();
                    var bB = fB.GetBody();
                    var typeA = bA.m_type;
                    var typeB = bB.m_type;
//                    b2Settings.b2Assert(typeA == b2Body.b2_dynamicBody || typeB == b2Body.b2_dynamicBody);
                    var activeA = bA.IsAwake() && typeA != b2Body.b2_staticBody;
                    var activeB = bB.IsAwake() && typeB != b2Body.b2_staticBody;
                    if (activeA == false && activeB == false) {
                        continue;
                    }
                    var collideA = bA.IsBullet() || typeA != b2Body.b2_dynamicBody;
                    var collideB = bB.IsBullet() || typeB != b2Body.b2_dynamicBody;
                    if (collideA == false && collideB == false) {
                        continue;
                    }
                    var alpha0 = bA.m_sweep.alpha0;
                    if (bA.m_sweep.alpha0 < bB.m_sweep.alpha0) {
                        alpha0 = bB.m_sweep.alpha0;
                        bA.m_sweep.Advance(alpha0);
                    } else if (bB.m_sweep.alpha0 < bA.m_sweep.alpha0) {
                        alpha0 = bA.m_sweep.alpha0;
                        bB.m_sweep.Advance(alpha0);
                    }
//                    b2Settings.b2Assert(alpha0 < 1.0);
                    var indexA = c.GetChildIndexA();
                    var indexB = c.GetChildIndexB();
                    var input = new b2TOIInput();
                    input.proxyA.Set(fA.GetShape(), indexA);
                    input.proxyB.Set(fB.GetShape(), indexB);
                    input.sweepA = bA.m_sweep;
                    input.sweepB = bB.m_sweep;
                    input.tMax = 1.0;
                    var output = new b2TOIOutput();
                    b2Collision.b2TimeOfImpact(output, input);
                    var beta = output.t;
                    if (output.state == b2TOIOutput.e_touching) {
                        alpha = b2Math.Min(alpha0 + (1.0 - alpha0) * beta, 1.0);
                    } else {
                        alpha = 1.0;
                    }
                    c.m_toi = alpha;
                    c.m_flags |= b2Contact.e_toiFlag;
                }
                if (alpha < minAlpha) {
                    minContact = c;
                    minAlpha = alpha;
                }
            }
            if (minContact == null || 1.0 - 10.0 * b2Settings.b2_epsilon < minAlpha) {
                this.m_stepComplete = true;
                break;
            }
            var fA = minContact.GetFixtureA();
            var fB = minContact.GetFixtureB();
            var bA = fA.GetBody();
            var bB = fB.GetBody();
            b2World.s_backupA.Set(bA.m_sweep);
            b2World.s_backupB.Set(bB.m_sweep);
            bA.Advance(minAlpha);
            bB.Advance(minAlpha);
            minContact.Update(this.m_contactManager.m_contactListener);
            minContact.m_flags &= ~b2Contact.e_toiFlag;
            ++minContact.m_toiCount;
            if (minContact.IsEnabled() == false || minContact.IsTouching() == false) {
                minContact.SetEnabled(false);
                bA.m_sweep.Set(b2World.s_backupA);
                bB.m_sweep.Set(b2World.s_backupB);
                bA.SynchronizeTransform();
                bB.SynchronizeTransform();
                continue;
            }
            bA.SetAwake(true);
            bB.SetAwake(true);
            island.Clear();
            island.AddBody(bA);
            island.AddBody(bB);
            island.AddContact(minContact);
            bA.m_flags |= b2Body.e_islandFlag;
            bB.m_flags |= b2Body.e_islandFlag;
            minContact.m_flags |= b2Contact.e_islandFlag;
            var bodies = new Array(bA, bB);
            for (var i = 0; i < 2; ++i) {
                b = bodies[i];
                if (b.m_type == b2Body.b2_dynamicBody) {
                    for (var ce = b.m_contactList; ce; ce = ce.next) {
                        if (island.m_bodyCount == island.m_bodyCapacity) {
                            break;
                        }
                        if (island.m_contactCount == island.m_contactCapacity) {
                            break;
                        }
                        var contact = ce.contact;
                        if (contact.m_flags & b2Contact.e_islandFlag) {
                            continue;
                        }
                        var other = ce.other;
                        if (other.m_type == b2Body.b2_dynamicBody && b.IsBullet() == false && other.IsBullet() == false) {
                            continue;
                        }
                        var sensorA = contact.m_fixtureA.m_isSensor;
                        var sensorB = contact.m_fixtureB.m_isSensor;
                        if (sensorA || sensorB) {
                            continue;
                        }
                        var backup = other.m_sweep.Copy();
                        if ((other.m_flags & b2Body.e_islandFlag) == 0) {
                            other.Advance(minAlpha);
                        }
                        contact.Update(this.m_contactManager.m_contactListener);
                        if (contact.IsEnabled() == false) {
                            other.m_sweep = backup.Copy();
                            other.SynchronizeTransform();
                            continue;
                        }
                        if (contact.IsTouching() == false) {
                            other.m_sweep = backup.Copy();
                            other.SynchronizeTransform();
                            continue;
                        }
                        contact.m_flags |= b2Contact.e_islandFlag;
                        island.AddContact(contact);
                        if (other.m_flags & b2Body.e_islandFlag) {
                            continue;
                        }
                        other.m_flags |= b2Body.e_islandFlag;
                        if (other.m_type != b2Body.b2_staticBody) {
                            other.SetAwake(true);
                        }
                        island.AddBody(other);
                    }
                }
            }
            var subStep = b2World.s_timestep;
            subStep.dt = (1.0 - minAlpha) * step.dt;
            subStep.inv_dt = 1.0 / subStep.dt;
            subStep.dtRatio = 1.0;
            subStep.positionIterations = 20;
            subStep.velocityIterations = step.velocityIterations;
            subStep.warmStarting = false;
            island.SolveTOI(subStep, bA.m_islandIndex, bB.m_islandIndex);
            for (var i = 0; i < island.m_bodyCount; ++i) {
                b = island.m_bodies[i];
                b.m_flags &= ~b2Body.e_islandFlag;
                if (b.m_type != b2Body.b2_dynamicBody) {
                    continue;
                }
                b.SynchronizeFixtures();
                for (var ce = b.m_contactList; ce; ce = ce.next) {
                    ce.contact.m_flags &= ~(b2Contact.e_toiFlag | b2Contact.e_islandFlag);
                }
            }
            this.m_contactManager.FindNewContacts();
            if (this.m_subStepping) {
                this.m_stepComplete = false;
                break;
            }
        }
    }
    b2World.prototype.DrawJoint = function(joint) {
        var b1 = joint.GetBodyA();
        var b2 = joint.GetBodyB();
        var xf1 = b1.m_xf;
        var xf2 = b2.m_xf;
        var x1 = xf1.p;
        var x2 = xf2.p;
        var p1 = joint.GetAnchorA();
        var p2 = joint.GetAnchorB();
        var color = b2World.s_jointColor;
        switch (joint.m_type) {
        case b2Joint.e_distanceJoint:
            this.m_debugDraw.DrawSegment(p1, p2, color);
            break;
        case b2Joint.e_pulleyJoint:
            {
                var pulley = ((joint instanceof b2PulleyJoint ? joint : null));
                var s1 = pulley.GetGroundAnchorA();
                var s2 = pulley.GetGroundAnchorB();
                this.m_debugDraw.DrawSegment(s1, p1, color);
                this.m_debugDraw.DrawSegment(s2, p2, color);
                this.m_debugDraw.DrawSegment(s1, s2, color);
            }
            break;
        case b2Joint.e_mouseJoint:
            this.m_debugDraw.DrawSegment(p1, p2, color);
            break;
        default:
            if (b1 != this.m_groundBody) this.m_debugDraw.DrawSegment(x1, p1, color);
            this.m_debugDraw.DrawSegment(p1, p2, color);
            if (b2 != this.m_groundBody) this.m_debugDraw.DrawSegment(x2, p2, color);
        }
    }
    b2World.prototype.DrawShape = function(shape, xf, color) {
        switch (shape.m_type) {
        case b2Shape.e_circle:
            {
                var circle = ((shape instanceof b2CircleShape ? shape : null));
                var center = b2Math.MulXV(xf, circle.m_p);
                var radius = circle.m_radius;
                var axis = new b2Vec2(xf.q.c, xf.q.s);
                this.m_debugDraw.DrawSolidCircle(center, radius, axis, color);
            }
            break;
        case b2Shape.e_polygon:
            {
                var i = 0;
                var poly = ((shape instanceof b2PolygonShape ? shape : null));
                var vertexCount = parseInt(poly.GetVertexCount());
                var localVertices = poly.m_vertices;
                var vertices = new Vector(vertexCount);
                for (i = 0; i < vertexCount; ++i) {
                    vertices[i] = b2Math.MulXV(xf, localVertices[i]);
                }
                this.m_debugDraw.DrawSolidPolygon(vertices, vertexCount, color);
            }
            break;
        case b2Shape.e_edge:
            {
                var edge = (shape instanceof b2EdgeShape ? shape : null);
                this.m_debugDraw.DrawSegment(b2Math.MulXV(xf, edge.GetVertex1()), b2Math.MulXV(xf, edge.GetVertex2()), color);
            }
            break;
        case b2Shape.e_chain:
            {
                var chain = ((shape instanceof b2ChainShape ? shape : null));
                var count = chain.m_count;
                var vertices = chain.m_vertices;
                var v1 = b2Math.MulXV(xf, vertices[0]);
                for (var i = 0; i < count; ++i) {
                    v2 = b2Math.MulXV(xf, vertices[i]);
                    this.m_debugDraw.DrawSegment(v1, v2, color);
                    this.m_debugDraw.DrawCircle(v1, 0.05, color);
                    v1.SetV(v2);
                }
            }
            break;
        }
    }
    Box2D.postDefs.push(function() {
        Box2D.Dynamics.b2World.s_timestep2 = new b2TimeStep();
        Box2D.Dynamics.b2World.s_xf = new b2Transform();
        Box2D.Dynamics.b2World.s_backupA = new b2Sweep();
        Box2D.Dynamics.b2World.s_backupB = new b2Sweep();
        Box2D.Dynamics.b2World.s_timestep = new b2TimeStep();
//        Box2D.Dynamics.b2World.s_queue = new Vector();
        Box2D.Dynamics.b2World.s_jointColor = new b2Color(0.5, 0.8, 0.8);
        Box2D.Dynamics.b2World.e_newFixture = 0x0001;
        Box2D.Dynamics.b2World.e_locked = 0x0002;
        Box2D.Dynamics.b2World.e_clearForces = 0x0004;
    });
})();
(function() {
    var b2CircleShape = Box2D.Collision.Shapes.b2CircleShape,
        b2EdgeShape = Box2D.Collision.Shapes.b2EdgeShape,
        b2ChainShape = Box2D.Collision.Shapes.b2ChainShape,
        b2MassData = Box2D.Collision.Shapes.b2MassData,
        b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape,
        b2Shape = Box2D.Collision.Shapes.b2Shape,
        b2ChainAndCircleContact = Box2D.Dynamics.Contacts.b2ChainAndCircleContact,
        b2ChainAndPolygonContact = Box2D.Dynamics.Contacts.b2ChainAndPolygonContact,
        b2CircleContact = Box2D.Dynamics.Contacts.b2CircleContact,
        b2Contact = Box2D.Dynamics.Contacts.b2Contact,
        b2ContactEdge = Box2D.Dynamics.Contacts.b2ContactEdge,
        b2ContactPositionConstraint = Box2D.Dynamics.Contacts.b2ContactPositionConstraint,
        b2ContactRegister = Box2D.Dynamics.Contacts.b2ContactRegister,
        b2ContactSolver = Box2D.Dynamics.Contacts.b2ContactSolver,
        b2ContactSolverDef = Box2D.Dynamics.Contacts.b2ContactSolverDef,
        b2ContactVelocityConstraint = Box2D.Dynamics.Contacts.b2ContactVelocityConstraint,
        b2EdgeAndCircleContact = Box2D.Dynamics.Contacts.b2EdgeAndCircleContact,
        b2PolygonAndCircleContact = Box2D.Dynamics.Contacts.b2PolygonAndCircleContact,
        b2EdgeAndPolygonContact = Box2D.Dynamics.Contacts.b2EdgeAndPolygonContact,
        b2PolygonContact = Box2D.Dynamics.Contacts.b2PolygonContact,
        b2PositionSolverManifold = Box2D.Dynamics.Contacts.b2PositionSolverManifold,
        b2VelocityConstraintPoint = Box2D.Dynamics.Contacts.b2VelocityConstraintPoint,
        b2Body = Box2D.Dynamics.b2Body,
        b2BodyDef = Box2D.Dynamics.b2BodyDef,
        b2ContactFilter = Box2D.Dynamics.b2ContactFilter,
        b2ContactImpulse = Box2D.Dynamics.b2ContactImpulse,
        b2ContactListener = Box2D.Dynamics.b2ContactListener,
        b2ContactManager = Box2D.Dynamics.b2ContactManager,
        b2Draw = Box2D.Dynamics.b2Draw,
        b2DestructionListener = Box2D.Dynamics.b2DestructionListener,
        b2Filter = Box2D.Dynamics.b2Filter,
        b2Fixture = Box2D.Dynamics.b2Fixture,
        b2FixtureDef = Box2D.Dynamics.b2FixtureDef,
        b2Island = Box2D.Dynamics.b2Island,
        b2Position = Box2D.Dynamics.b2Position,
        b2Profile = Box2D.Dynamics.b2Profile,
        b2SolverData = Box2D.Dynamics.b2SolverData,
        b2TimeStep = Box2D.Dynamics.b2TimeStep,
        b2Velocity = Box2D.Dynamics.b2Velocity,
        b2World = Box2D.Dynamics.b2World,
        b2Color = Box2D.Common.b2Color,
        b2Settings = Box2D.Common.b2Settings,
        b2Mat22 = Box2D.Common.Math.b2Mat22,
        b2Mat33 = Box2D.Common.Math.b2Mat33,
        b2Rot = Box2D.Common.Math.b2Rot,
        b2Math = Box2D.Common.Math.b2Math,
        b2Sweep = Box2D.Common.Math.b2Sweep,
        b2Transform = Box2D.Common.Math.b2Transform,
        b2Vec2 = Box2D.Common.Math.b2Vec2,
        b2Vec3 = Box2D.Common.Math.b2Vec3,
        b2AABB = Box2D.Collision.b2AABB,
        b2Collision = Box2D.Collision.b2Collision,
        b2ContactID = Box2D.Collision.b2ContactID,
        b2Distance = Box2D.Collision.b2Distance,
        b2DistanceInput = Box2D.Collision.b2DistanceInput,
        b2DistanceOutput = Box2D.Collision.b2DistanceOutput,
        b2DistanceProxy = Box2D.Collision.b2DistanceProxy,
        b2DynamicTree = Box2D.Collision.b2DynamicTree,
        b2BroadPhase = Box2D.Collision.b2BroadPhase,
        b2TreeNode = Box2D.Collision.b2TreeNode,
        b2Pair = Box2D.Collision.b2Pair,
        b2Manifold = Box2D.Collision.b2Manifold,
        b2ManifoldPoint = Box2D.Collision.b2ManifoldPoint,
        b2RayCastInput = Box2D.Collision.b2RayCastInput,
        b2RayCastOutput = Box2D.Collision.b2RayCastOutput,
        b2SeparationFunction = Box2D.Collision.b2SeparationFunction,
        b2Simplex = Box2D.Collision.b2Simplex,
        b2SimplexCache = Box2D.Collision.b2SimplexCache,
        b2SimplexVertex = Box2D.Collision.b2SimplexVertex,
        b2TOIInput = Box2D.Collision.b2TOIInput,
        b2TOIOutput = Box2D.Collision.b2TOIOutput,
        b2WorldManifold = Box2D.Collision.b2WorldManifold,
        b2ClipVertex = Box2D.Collision.b2ClipVertex,
        b2ContactFeature = Box2D.Collision.b2ContactFeature,
        IBroadPhase = Box2D.Collision.IBroadPhase;

    Box2D.inherit(b2ChainAndCircleContact, Box2D.Dynamics.Contacts.b2Contact);
    b2ChainAndCircleContact.prototype.__super = Box2D.Dynamics.Contacts.b2Contact.prototype;
    b2ChainAndCircleContact.b2ChainAndCircleContact = function() {
        Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply(this, arguments);
    };
    b2ChainAndCircleContact.Create = function(allocator) {
        return new b2ChainAndCircleContact();
    }
    b2ChainAndCircleContact.Destroy = function(contact, allocator) {}
    b2ChainAndCircleContact.prototype.Reset = function(fixtureA, indexA, fixtureB, indexB) {
        this.__super.Reset.call(this, fixtureA, indexA, fixtureB, indexB);
//        b2Settings.b2Assert(this.m_fixtureA.GetType() === b2Shape.e_chain);
//        b2Settings.b2Assert(this.m_fixtureB.GetType() === b2Shape.e_circle);
    }
    b2ChainAndCircleContact.prototype.Evaluate = function(manifold, xfA, xfB) {
        var chain = this.m_fixtureA.GetShape() instanceof b2ChainShape ? this.m_fixtureA.GetShape() : null;
        var edge = new b2EdgeShape;
        chain.GetChildEdge(edge, this.m_indexA);
        b2Collision.CollideEdgeAndCircle(manifold, edge, xfA, (this.m_fixtureB.GetShape() instanceof b2CircleShape ? this.m_fixtureB.GetShape() : null), xfB);
    }

    Box2D.inherit(b2ChainAndPolygonContact, Box2D.Dynamics.Contacts.b2Contact);
    b2ChainAndPolygonContact.prototype.__super = Box2D.Dynamics.Contacts.b2Contact.prototype;
    b2ChainAndPolygonContact.b2ChainAndPolygonContact = function() {
        Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply(this, arguments);
    };
    b2ChainAndPolygonContact.Create = function(allocator) {
        return new b2ChainAndPolygonContact();
    }
    b2ChainAndPolygonContact.Destroy = function(contact, allocator) {}
    b2ChainAndPolygonContact.prototype.Reset = function(fixtureA, indexA, fixtureB, indexB) {
        this.__super.Reset.call(this, fixtureA, indexA, fixtureB, indexB);
        b2Settings.b2Assert(this.m_fixtureA.GetType() === b2Shape.e_chain);
        b2Settings.b2Assert(this.m_fixtureB.GetType() === b2Shape.e_polygon);
    }
    b2ChainAndPolygonContact.prototype.Evaluate = function(manifold, xfA, xfB) {
        var chain = this.m_fixtureA.GetShape() instanceof b2ChainShape ? this.m_fixtureA.GetShape() : null;
        var edge = new b2EdgeShape;
        chain.GetChildEdge(edge, this.m_indexA);
        b2Collision.CollideEdgeAndPolygon(manifold, edge, xfA, (this.m_fixtureB.GetShape() instanceof b2PolygonShape ? this.m_fixtureB.GetShape() : null), xfB);
    }

    Box2D.inherit(b2CircleContact, Box2D.Dynamics.Contacts.b2Contact);
    b2CircleContact.prototype.__super = Box2D.Dynamics.Contacts.b2Contact.prototype;
    b2CircleContact.b2CircleContact = function() {
        Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply(this, arguments);
    };
    b2CircleContact.Create = function(allocator) {
        return new b2CircleContact();
    }
    b2CircleContact.Destroy = function(contact, allocator) {}
    b2CircleContact.prototype.Reset = function(fixtureA, indexA, fixtureB, indexB) {
        this.__super.Reset.call(this, fixtureA, indexA, fixtureB, indexB);
    }
    b2CircleContact.prototype.Evaluate = function(manifold, xfA, xfB) {
        b2Collision.CollideCircles(manifold, (this.m_fixtureA.GetShape() instanceof b2CircleShape ? this.m_fixtureA.GetShape() : null), xfA, (this.m_fixtureB.GetShape() instanceof b2CircleShape ? this.m_fixtureB.GetShape() : null), xfB);
    }
    b2Contact.b2Contact = function() {
        this.m_nodeA = new b2ContactEdge();
        this.m_nodeB = new b2ContactEdge();
        this.m_manifold = new b2Manifold();
        this.m_oldManifold = new b2Manifold();
        this.m_fixtureA = null;
        this.m_fixtureB = null;
    };
    b2Contact.InitializeRegisters = function() {
        b2Contact.s_registers = new Vector(b2Shape.e_typeCount);
        for (var i = 0; i < b2Shape.e_typeCount; i++) {
            b2Contact.s_registers[i] = new Vector(b2Shape.e_typeCount);
            for (var j = 0; j < b2Shape.e_typeCount; j++) {
                b2Contact.s_registers[i][j] = new b2ContactRegister();
            }
        }
        b2Contact.AddType(b2CircleContact.Create, b2CircleContact.Destroy, b2Shape.e_circle, b2Shape.e_circle);
        b2Contact.AddType(b2PolygonAndCircleContact.Create, b2PolygonAndCircleContact.Destroy, b2Shape.e_polygon, b2Shape.e_circle);
        b2Contact.AddType(b2PolygonContact.Create, b2PolygonContact.Destroy, b2Shape.e_polygon, b2Shape.e_polygon);
        b2Contact.AddType(b2EdgeAndCircleContact.Create, b2EdgeAndCircleContact.Destroy, b2Shape.e_edge, b2Shape.e_circle);
        b2Contact.AddType(b2EdgeAndPolygonContact.Create, b2EdgeAndPolygonContact.Destroy, b2Shape.e_edge, b2Shape.e_polygon);
        b2Contact.AddType(b2ChainAndCircleContact.Create, b2ChainAndCircleContact.Destroy, b2Shape.e_chain, b2Shape.e_circle);
        b2Contact.AddType(b2ChainAndPolygonContact.Create, b2ChainAndPolygonContact.Destroy, b2Shape.e_chain, b2Shape.e_polygon);
    }
    b2Contact.AddType = function(createFcn, destoryFcn, type1, type2) {
        b2Settings.b2Assert(0 <= type1 && type1 < b2Shape.e_typeCount);
        b2Settings.b2Assert(0 <= type2 && type2 < b2Shape.e_typeCount);

        b2Contact.s_registers[type1][type2].createFcn = createFcn;
        b2Contact.s_registers[type1][type2].destroyFcn = destoryFcn;
        b2Contact.s_registers[type1][type2].primary = true;

        if (type1 != type2) {
            b2Contact.s_registers[type2][type1].createFcn = createFcn;
            b2Contact.s_registers[type2][type1].destroyFcn = destoryFcn;
            b2Contact.s_registers[type2][type1].primary = false;
        }
    }
    b2Contact.Create = function(fixtureA, indexA, fixtureB, indexB, allocator) {
        if (b2Contact.s_initialized == false) {
            b2Contact.InitializeRegisters();
            b2Contact.s_initialized = true;
        }

        var type1 = fixtureA.GetType();
        var type2 = fixtureB.GetType();

        b2Settings.b2Assert(0 <= type1 && type1 < b2Shape.e_typeCount);
        b2Settings.b2Assert(0 <= type2 && type2 < b2Shape.e_typeCount);

        var reg = this.s_registers[type1][type2];
        var c;
        if (reg.pool) {
            c = reg.pool;
            reg.pool = c.m_next;
            reg.poolCount--;
            c.Reset(fixtureA, indexA, fixtureB, indexB);
            return c;
        }
        var createFcn = reg.createFcn;
        if (createFcn) {
            if (reg.primary) {
                c = createFcn(allocator);
                c.Reset(fixtureA, indexA, fixtureB, indexB);
            } else {
                c = createFcn(allocator);
                c.Reset(fixtureB, indexB, fixtureA, indexA);
            }
            return c;
        } else {
            return null;
        }
    }
    b2Contact.Destroy = function(contact, allocator) {
        b2Settings.b2Assert(b2Contact.s_initialized == true);

        if (contact.m_manifold.pointCount > 0) {
            contact.GetFixtureA().GetBody().SetAwake(true);
            contact.GetFixtureB().GetBody().SetAwake(true);
        }

        var typeA = contact.GetFixtureA().GetType();
        var typeB = contact.GetFixtureB().GetType();

        b2Settings.b2Assert(0 <= typeA && typeB < b2Shape.e_typeCount);
        b2Settings.b2Assert(0 <= typeA && typeB < b2Shape.e_typeCount);

        var reg = this.s_registers[typeA][typeB];
        if (true) {
            reg.poolCount++;
            contact.m_next = reg.pool;
            reg.pool = contact;
        }
        var destroyFcn = reg.destroyFcn;
        destroyFcn(contact, allocator);
    }
    b2Contact.prototype.b2Contact = function() {}
    b2Contact.prototype.Reset = function(fixtureA, indexA, fixtureB, indexB) {
        this.m_flags = b2Contact.e_enabledFlag;

        this.m_fixtureA = fixtureA;
        this.m_fixtureB = fixtureB;

        this.m_indexA = indexA;
        this.m_indexB = indexB;

        this.m_manifold.pointCount = 0;

        this.m_prev = null;
        this.m_next = null;

        this.m_nodeA.contact = null;
        this.m_nodeA.prev = null;
        this.m_nodeA.next = null;
        this.m_nodeA.other = null;

        this.m_nodeB.contact = null;
        this.m_nodeB.prev = null;
        this.m_nodeB.next = null;
        this.m_nodeB.other = null;

        this.m_toiCount = 0;
        this.m_friction = b2Settings.b2MixFriction(this.m_fixtureA.m_friction, this.m_fixtureB.m_friction);
        this.m_restitution = b2Settings.b2MixRestitution(this.m_fixtureA.m_restitution, this.m_fixtureB.m_restitution);
    }
    b2Contact.prototype.GetFriction = function() {
        return this.m_friction;
    }
    b2Contact.prototype.SetFriction = function(friction) {
        this.m_friction = friction;
    }
    b2Contact.prototype.ResetFriction = function() {
        this.m_friction = b2Settings.b2MixFriction(this.m_fixtureA.m_friction, this.m_fixtureB.m_friction);
    }
    b2Contact.prototype.GetRestitution = function() {
        return this.m_restitution;
    }
    b2Contact.prototype.SetRestitution = function(restitution) {
        this.m_restitution = restitution;
    }
    b2Contact.prototype.ResetRestitution = function() {
        this.m_restitution = b2Settings.b2MixRestitution(this.m_fixtureA.m_restitution, this.m_fixtureB.m_restitution);
    }
    b2Contact.prototype.GetManifold = function() {
        return this.m_manifold;
    }
    b2Contact.prototype.GetWorldManifold = function(worldManifold) {
        var bodyA = this.m_fixtureA.GetBody();
        var bodyB = this.m_fixtureB.GetBody();
        var shapeA = this.m_fixtureA.GetShape();
        var shapeB = this.m_fixtureB.GetShape();
        worldManifold.Initialize(this.m_manifold, bodyA.GetTransform(), shapeA.m_radius, bodyB.GetTransform(), shapeB.m_radius);
    }
    b2Contact.prototype.IsTouching = function() {
        return (this.m_flags & b2Contact.e_touchingFlag) == b2Contact.e_touchingFlag;
    }
    /*
    b2Contact.prototype.IsContinuous = function() {
        return (this.m_flags & b2Contact.e_continuousFlag) == b2Contact.e_continuousFlag;
    }
    b2Contact.prototype.SetSensor = function(sensor) {
        if (sensor) {
            this.m_flags |= b2Contact.e_sensorFlag;
        } else {
            this.m_flags &= ~b2Contact.e_sensorFlag;
        }
    }
    */
    b2Contact.prototype.SetEnabled = function(flag) {
        if (flag) {
            this.m_flags |= b2Contact.e_enabledFlag;
        } else {
            this.m_flags &= ~b2Contact.e_enabledFlag;
        }
    }
    b2Contact.prototype.IsEnabled = function() {
        return (this.m_flags & b2Contact.e_enabledFlag) == b2Contact.e_enabledFlag;
    }
    b2Contact.prototype.GetNext = function() {
        return this.m_next;
    }
    b2Contact.prototype.GetFixtureA = function() {
        return this.m_fixtureA;
    }
    b2Contact.prototype.GetFixtureB = function() {
        return this.m_fixtureB;
    }
    b2Contact.prototype.FlagForFiltering = function() {
        this.m_flags |= b2Contact.e_filterFlag;
    }
    b2Contact.prototype.Update = function(listener) {
        var tManifold = this.m_oldManifold;
        this.m_oldManifold = this.m_manifold;
        this.m_manifold = tManifold;
        this.m_flags |= b2Contact.e_enabledFlag;
        var touching = false;
        var wasTouching = (this.m_flags & b2Contact.e_touchingFlag) == b2Contact.e_touchingFlag;

        var sensorA = this.m_fixtureA.IsSensor();
        var sensorB = this.m_fixtureB.IsSensor();
        var sensor = sensorA || sensorB;

        var bodyA = this.m_fixtureA.m_body;
        var bodyB = this.m_fixtureB.m_body;
        var xfA = bodyA.GetTransform();
        var xfB = bodyB.GetTransform();

        if (sensor) {
            var shapeA = this.m_fixtureA.GetShape();
            var shapeB = this.m_fixtureB.GetShape();
            touching = b2Shape.TestOverlap(shapeA, this.m_indexA, shapeB, this.m_indexB, xfA, xfB);
            this.m_manifold.pointCount = 0;
        } else {
            this.Evaluate(this.m_manifold, xfA, xfB);
            touching = this.m_manifold.pointCount > 0;
            for (var i = 0; i < this.m_manifold.pointCount; ++i) {
                var mp2 = this.m_manifold.points[i];
                mp2.normalImpulse = 0.0;
                mp2.tangentImpulse = 0.0;
                var id2 = mp2.id;
                for (var j = 0; j < this.m_oldManifold.pointCount; ++j) {
                    var mp1 = this.m_oldManifold.points[j];
                    if (mp1.id.key == id2.key) {
                        mp2.normalImpulse = mp1.normalImpulse;
                        mp2.tangentImpulse = mp1.tangentImpulse;
                        break;
                    }
                }
            }
            if (touching != wasTouching) {
                bodyA.SetAwake(true);
                bodyB.SetAwake(true);
            }
        }
        if (touching) {
            this.m_flags |= b2Contact.e_touchingFlag;
        } else {
            this.m_flags &= ~b2Contact.e_touchingFlag;
        }
        if (wasTouching == false && touching == true && listener) {
            listener.BeginContact(this);
        }
        if (wasTouching == true && touching == false && listener) {
            listener.EndContact(this);
        }
        if (sensor == false && touching && listener) {
            listener.PreSolve(this, this.m_oldManifold);
        }
    }
    b2Contact.prototype.Evaluate = function(manifold, xfA, xfB) {}
    b2Contact.prototype.GetChildIndexA = function() {
        return this.m_indexA;
    }
    b2Contact.prototype.GetChildIndexB = function() {
        return this.m_indexB;
    }
    Box2D.postDefs.push(function() {
        Box2D.Dynamics.Contacts.b2Contact.e_islandFlag      = 0x0001;
        Box2D.Dynamics.Contacts.b2Contact.e_touchingFlag    = 0x0002;
        Box2D.Dynamics.Contacts.b2Contact.e_enabledFlag     = 0x0004;
        Box2D.Dynamics.Contacts.b2Contact.e_filterFlag      = 0x0008;
        Box2D.Dynamics.Contacts.b2Contact.e_bulletHitFlag   = 0x0010;
        Box2D.Dynamics.Contacts.b2Contact.e_toiFlag         = 0x0020;
        Box2D.Dynamics.Contacts.b2Contact.s_initialized = false;
        Box2D.Dynamics.Contacts.b2Contact.s_registers = null;
    });
    b2ContactEdge.b2ContactEdge = function() {};
    b2ContactPositionConstraint.b2ContactPositionConstraint = function() {
        this.localNormal = new b2Vec2();
        this.localPoint = new b2Vec2();
        this.localCenterA = new b2Vec2();
        this.localCenterB = new b2Vec2();
    };
    b2ContactPositionConstraint.prototype.b2ContactPositionConstraint = function() {
        this.localPoints = new Vector(b2Settings.b2_maxManifoldPoints);
        for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
            this.localPoints[i] = new b2Vec2();
        }
    }
    b2ContactRegister.b2ContactRegister = function() {
        this.poolCount = 0;
    };
    b2ContactSolver.b2ContactSolver = function() {
        this.m_step = new b2TimeStep();
        this.m_positionConstraints = new Vector();
        this.m_velocityConstraints = new Vector();
        this.m_contacts = null;
    };
    b2ContactSolver.prototype.b2ContactSolver = function() {}
    b2ContactSolver.prototype.Initialize = function(def) {
        this.m_step.Set(def.step);
        this.m_allocator = def.allocator;
        this.m_count = def.count;
        var contact;
        var i = 0;
        var pc;
        var vc;
        var cp;
        var vcp;
        while (this.m_positionConstraints.length < this.m_count) {
            this.m_positionConstraints[this.m_positionConstraints.length] = new b2ContactPositionConstraint();
        }
        while (this.m_velocityConstraints.length < this.m_count) {
            this.m_velocityConstraints[this.m_velocityConstraints.length] = new b2ContactVelocityConstraint();
        }
        this.m_positions = def.positions;
        this.m_velocities = def.velocities;
        this.m_contacts = def.contacts;

        // Initialize position independent portions of the constraints.
        for (i = 0; i < this.m_count; ++i) {
            contact = this.m_contacts[i];

            var fixtureA = contact.m_fixtureA;
            var fixtureB = contact.m_fixtureB;
            var shapeA = fixtureA.GetShape();
            var shapeB = fixtureB.GetShape();
            var radiusA = shapeA.m_radius;
            var radiusB = shapeB.m_radius;
            var bodyA = fixtureA.GetBody();
            var bodyB = fixtureB.GetBody();
            var manifold = contact.GetManifold();

            var pointCount = manifold.pointCount;
            b2Settings.b2Assert(pointCount > 0);

            vc = this.m_velocityConstraints[i];
            vc.friction = contact.m_friction;
            vc.restitution = contact.m_restitution;
            vc.indexA = bodyA.m_islandIndex;
            vc.indexB = bodyB.m_islandIndex;
            vc.invMassA = bodyA.m_invMass;
            vc.invMassB = bodyB.m_invMass;
            vc.invIA = bodyA.m_invI;
            vc.invIB = bodyB.m_invI;
            vc.contactIndex = i;
            vc.pointCount = pointCount;
            vc.K.SetZero();
            vc.normalMass.SetZero();

            pc = this.m_positionConstraints[i];
            pc.indexA = bodyA.m_islandIndex;
            pc.indexB = bodyB.m_islandIndex;
            pc.invMassA = bodyA.m_invMass;
            pc.invMassB = bodyB.m_invMass;
            pc.localCenterA.SetV(bodyA.m_sweep.localCenter);
            pc.localCenterB.SetV(bodyB.m_sweep.localCenter);
            pc.invIA = bodyA.m_invI;
            pc.invIB = bodyB.m_invI;
            pc.localNormal.x = manifold.localNormal.x;
            pc.localNormal.y = manifold.localNormal.y;
            pc.localPoint.x = manifold.localPoint.x;
            pc.localPoint.y = manifold.localPoint.y;
            pc.pointCount = pointCount;
            pc.radiusA = radiusA;
            pc.radiusB = radiusB;
            pc.type = manifold.type;

            for (var j = 0; j < pointCount; ++j) {
                cp = manifold.points[j];
                vcp = vc.points[j];

                if (this.m_step.warmStarting) {
                    vcp.normalImpulse = this.m_step.dtRatio * cp.normalImpulse;
                    vcp.tangentImpulse = this.m_step.dtRatio * cp.tangentImpulse;
                } else {
                    vcp.normalImpulse = 0.0;
                    vcp.tangentImpulse = 0.0;
                }

                vcp.rA.SetZero();
                vcp.rB.SetZero();
                vcp.normalMass = 0.0;
                vcp.tangentMass = 0.0;
                vcp.velocityBias = 0.0;

                pc.localPoints[j].x = cp.localPoint.x;
                pc.localPoints[j].y = cp.localPoint.y;
            }
        }
    }
    b2ContactSolver.prototype.InitializeVelocityConstraints = function() {
        var i = 0;
        var j = 0;
        var vc, vcp;
        var pc;
        for (i = 0; i < this.m_count; ++i) {
            vc = this.m_velocityConstraints[i];
            pc = this.m_positionConstraints[i];
            var radiusA = pc.radiusA;
            var radiusB = pc.radiusB;
            var manifold = this.m_contacts[vc.contactIndex].GetManifold();
            var indexA = vc.indexA;
            var indexB = vc.indexB;
            var mA = vc.invMassA;
            var mB = vc.invMassB;
            var iA = vc.invIA;
            var iB = vc.invIB;
            var localCenterA = pc.localCenterA;
            var localCenterB = pc.localCenterB;
            var cA = this.m_positions[indexA].c;
            var aA = this.m_positions[indexA].a;
            var vA = this.m_velocities[indexA].v;
            var wA = this.m_velocities[indexA].w;
            var cB = this.m_positions[indexB].c;
            var aB = this.m_positions[indexB].a;
            var vB = this.m_velocities[indexB].v;
            var wB = this.m_velocities[indexB].w;
            b2Settings.b2Assert(manifold.pointCount > 0);
            var xfA = b2ContactSolver.s_xfA;
            var xfB = b2ContactSolver.s_xfB;
            xfA.q.Set(aA);
            xfB.q.Set(aB);
            xfA.p.x = cA.x - (xfA.q.c * localCenterA.x - xfA.q.s * localCenterA.y);
            xfA.p.y = cA.y - (xfA.q.s * localCenterA.x + xfA.q.c * localCenterA.y);
            xfB.p.x = cB.x - (xfB.q.c * localCenterB.x - xfB.q.s * localCenterB.y);
            xfB.p.y = cB.y - (xfB.q.s * localCenterB.x + xfB.q.c * localCenterB.y);
            var worldManifold = b2ContactSolver.s_b2WorldManifold;
            worldManifold.Initialize(manifold, xfA, radiusA, xfB, radiusB);
            vc.normal.SetV(worldManifold.normal);
            var pointCount = vc.pointCount;
            for (j = 0; j < pointCount; ++j) {
                vcp = vc.points[j];
                var vcprA = vcp.rA;
                var vcprB = vcp.rB;
                var vcnormal = vc.normal;
                vcprA.x = worldManifold.points[j].x - cA.x;
                vcprA.y = worldManifold.points[j].y - cA.y;
                vcprB.x = worldManifold.points[j].x - cB.x;
                vcprB.y = worldManifold.points[j].y - cB.y;
                var rnA = vcprA.x * vcnormal.y - vcprA.y * vcnormal.x;
                var rnB = vcprB.x * vcnormal.y - vcprB.y * vcnormal.x;
                var kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                vcp.normalMass = kNormal > 0.0 ? 1.0 / kNormal : 0.0;
                var tangentX = vcnormal.y
                var tangentY = -vcnormal.x;
                var rtA = vcprA.x * tangentY - vcprA.y * tangentX;
                var rtB = vcprB.x * tangentY - vcprB.y * tangentX;
                var kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
                vcp.tangentMass = kTangent > 0.0 ? 1.0 /  kTangent : 0.0;
                vcp.velocityBias = 0.0;
                var tmpX = vB.x + (-wB) * vcprB.y;
                var tmpY = vB.y + wB * vcprB.x;
                tmpX -= vA.x;
                tmpY -= vA.y;
                tmpX -= (-wA) * vcprA.y;
                tmpY -= wA * vcprA.x;
                var vRel = vcnormal.x * tmpX + vcnormal.y * tmpY;
                if (vRel < -b2Settings.b2_velocityThreshold) {
                    vcp.velocityBias = -vc.restitution * vRel;
                }
            }
            if (vc.pointCount == 2) {
                var vcp1 = vc.points[0];
                var vcp2 = vc.points[1];
                var rn1A = b2Math.CrossVV(vcp1.rA, vc.normal);
                var rn1B = b2Math.CrossVV(vcp1.rB, vc.normal);
                var rn2A = b2Math.CrossVV(vcp2.rA, vc.normal);
                var rn2B = b2Math.CrossVV(vcp2.rB, vc.normal);
                var k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
                var k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
                var k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;
                var k_maxConditionNumber = 1000.0;
                if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12)) {
                    vc.K.ex.Set(k11, k12);
                    vc.K.ey.Set(k12, k22);
                    vc.K.GetInverse2(vc.normalMass);
                } else {
                    vc.pointCount = 1;
                }
            }
        }
    }
    b2ContactSolver.prototype.WarmStart = function() {
        // Warm start.
        var vcp;
        var PX, PY;
        for (var i = 0; i < this.m_count; ++i) {
            var vc = this.m_velocityConstraints[i];

            var indexA = vc.indexA;
            var indexB = vc.indexB;
            var mA = vc.invMassA;
            var iA = vc.invIA;
            var mB = vc.invMassB;
            var iB = vc.invIB;
            var pointCount = vc.pointCount;

            var vA = this.m_velocities[indexA].v;
            var wA = this.m_velocities[indexA].w;
            var vB = this.m_velocities[indexB].v;
            var wB = this.m_velocities[indexB].w;

            var normalX = vc.normal.x;
            var normalY = vc.normal.y;
            var tangentX = normalY;
            var tangentY = (-normalX);

            for (var j = 0; j < pointCount; ++j) {
                vcp = vc.points[j];
                PX = vcp.normalImpulse * normalX + vcp.tangentImpulse * tangentX;
                PY = vcp.normalImpulse * normalY + vcp.tangentImpulse * tangentY;
                wA -= iA * (vcp.rA.x * PY - vcp.rA.y * PX);
                vA.x -= mA * PX;
                vA.y -= mA * PY;
                wB += iB * (vcp.rB.x * PY - vcp.rB.y * PX);
                vB.x += mB * PX;
                vB.y += mB * PY;
            }

//            this.m_velocities[indexA].v = vA;
            this.m_velocities[indexA].w = wA;
//            this.m_velocities[indexB].v = vB;
            this.m_velocities[indexB].w = wB;
        }
    }
    b2ContactSolver.prototype.SolveVelocityConstraints = function() {
        var i = 0;
        var j = 0;
        var vcp;
        var dvX = 0;
        var dvY = 0;
        var vt;
        var vn;
        var lambda;
        var maxFriction;
        var newImpulse;
        var PX;
        var PY;
        var dX;
        var dY;
        var P1X;
        var P1Y;
        var P2X;
        var P2Y;
        var tMat;
        var tVec;
        for (i = 0; i < this.m_count; ++i) {
            var vc = this.m_velocityConstraints[i];
            var indexA = vc.indexA;
            var indexB = vc.indexB;
            var mA = vc.invMassA;
            var iA = vc.invIA;
            var mB = vc.invMassB;
            var iB = vc.invIB;
            var pointCount = vc.pointCount;
            var vA = this.m_velocities[indexA].v;
            var wA = this.m_velocities[indexA].w;
            var vB = this.m_velocities[indexB].v;
            var wB = this.m_velocities[indexB].w;
            var normalX = vc.normal.x;
            var normalY = vc.normal.y;
            var tangentX = normalY;
            var tangentY = (-normalX);
            var friction = vc.friction;
//            b2Settings.b2Assert(pointCount == 1 || pointCount == 2);
            for (j = 0; j < pointCount; ++j) {
                vcp = vc.points[j];
                dvX = vB.x - wB * vcp.rB.y -vA.x + wA * vcp.rA.y;
                dvY = vB.y + wB * vcp.rB.x - vA.y - wA * vcp.rA.x;
                vt = dvX * tangentX + dvY * tangentY;
                lambda = vcp.tangentMass * (-vt);
                maxFriction = friction * vcp.normalImpulse;
                newImpulse = b2Math.Clamp(vcp.tangentImpulse + lambda, -maxFriction, maxFriction);
                lambda = newImpulse - vcp.tangentImpulse;
                vcp.tangentImpulse = newImpulse;
                PX = lambda * tangentX;
                PY = lambda * tangentY;
                vA.x -= mA * PX;
                vA.y -= mA * PY;
                wA -= iA * (vcp.rA.x * PY - vcp.rA.y * PX);
                vB.x += mB * PX;
                vB.y += mB * PY;
                wB += iB * (vcp.rB.x * PY - vcp.rB.y * PX);
            }
            if (vc.pointCount == 1) {
                vcp = vc.points[0];
                dvX = vB.x + (-wB * vcp.rB.y) - vA.x - (-wA * vcp.rA.y);
                dvY = vB.y + (wB * vcp.rB.x) - vA.y - (wA * vcp.rA.x);
                vn = dvX * normalX + dvY * normalY;
                lambda = -vcp.normalMass * (vn - vcp.velocityBias);
                newImpulse = b2Math.Max(vcp.normalImpulse + lambda, 0.0);
                lambda = newImpulse - vcp.normalImpulse;
                vcp.normalImpulse = newImpulse;
                PX = lambda * normalX;
                PY = lambda * normalY;
                vA.x -= mA * PX;
                vA.y -= mA * PY;
                wA -= iA * (vcp.rA.x * PY - vcp.rA.y * PX);
                vB.x += mB * PX;
                vB.y += mB * PY;
                wB += iB * (vcp.rB.x * PY - vcp.rB.y * PX);
            } else {
                var cp1 = vc.points[0];
                var cp2 = vc.points[1];
                var aX = cp1.normalImpulse;
                var aY = cp2.normalImpulse;
//                b2Settings.b2Assert(aX >= 0.0 && aY >= 0.0);
                var dv1X = vB.x - wB * cp1.rB.y - vA.x + wA * cp1.rA.y;
                var dv1Y = vB.y + wB * cp1.rB.x - vA.y - wA * cp1.rA.x;
                var dv2X = vB.x - wB * cp2.rB.y - vA.x + wA * cp2.rA.y;
                var dv2Y = vB.y + wB * cp2.rB.x - vA.y - wA * cp2.rA.x;
                var vn1 = dv1X * normalX + dv1Y * normalY;
                var vn2 = dv2X * normalX + dv2Y * normalY;
                var bX = vn1 - cp1.velocityBias;
                var bY = vn2 - cp2.velocityBias;
                tMat = vc.K;
                bX -= tMat.ex.x * aX + tMat.ey.x * aY;
                bY -= tMat.ex.y * aX + tMat.ey.y * aY;
                for (;;) {
                    tMat = vc.normalMass;
                    var xX = (-(tMat.ex.x * bX + tMat.ey.x * bY));
                    var xY = (-(tMat.ex.y * bX + tMat.ey.y * bY));
                    if (xX >= 0.0 && xY >= 0.0) {
                        dX = xX - aX;
                        dY = xY - aY;
                        P1X = dX * normalX;
                        P1Y = dX * normalY;
                        P2X = dY * normalX;
                        P2Y = dY * normalY;
                        vA.x -= mA * (P1X + P2X);
                        vA.y -= mA * (P1Y + P2Y);
                        wA -= iA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
                        vB.x += mB * (P1X + P2X);
                        vB.y += mB * (P1Y + P2Y);
                        wB += iB * ((cp1.rB.x * P1Y - cp1.rB.y * P1X) + (cp2.rB.x * P2Y- cp2.rB.y * P2X));
                        cp1.normalImpulse = xX;
                        cp2.normalImpulse = xY;
                        break;
                    }
                    xX = - cp1.normalMass * bX;
                    xY = 0.0;
                    vn1 = 0.0;
                    vn2 = vc.K.ex.y * xX + bY;
                    if (xX >= 0.0 && vn2 >= 0.0) {
                        dX = xX - aX;
                        dY = xY - aY;
                        P1X = dX * normalX;
                        P1Y = dX * normalY;
                        P2X = dY * normalX;
                        P2Y = dY * normalY;
                        vA.x -= mA * (P1X + P2X);
                        vA.y -= mA * (P1Y + P2Y);
                        wA -= iA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
                        vB.x += mB * (P1X + P2X);
                        vB.y += mB * (P1Y + P2Y);
                        wB += iB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
                        cp1.normalImpulse = xX;
                        cp2.normalImpulse = xY;
                        break;
                    }
                    xX = 0.0;
                    xY = - cp2.normalMass * bY;
                    vn1 = vc.K.ey.x * xY + bX;
                    vn2 = 0.0;
                    if (xY >= 0.0 && vn1 >= 0.0) {
                        dX = xX - aX;
                        dY = xY - aY;
                        P1X = dX * normalX;
                        P1Y = dX * normalY;
                        P2X = dY * normalX;
                        P2Y = dY * normalY;
                        vA.x -= mA * (P1X + P2X);
                        vA.y -= mA * (P1Y + P2Y);
                        wA -= iA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
                        vB.x += mB * (P1X + P2X);
                        vB.y += mB * (P1Y + P2Y);
                        wB += iB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
                        cp1.normalImpulse = xX;
                        cp2.normalImpulse = xY;
                        break;
                    }
                    xX = 0.0;
                    xY = 0.0;
                    vn1 = bX;
                    vn2 = bY;
                    if (vn1 >= 0.0 && vn2 >= 0.0 ) {
                        dX = xX - aX;
                        dY = xY - aY;
                        P1X = dX * normalX;
                        P1Y = dX * normalY;
                        P2X = dY * normalX;
                        P2Y = dY * normalY;
                        vA.x -= mA * (P1X + P2X);
                        vA.y -= mA * (P1Y + P2Y);
                        wA -= iA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
                        vB.x += mB * (P1X + P2X);
                        vB.y += mB * (P1Y + P2Y);
                        wB += iB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
                        cp1.normalImpulse = xX;
                        cp2.normalImpulse = xY;
                        break;
                    }
                    break;
                }
            }
//            this.m_velocities[indexA].v = vA.Copy();
            this.m_velocities[indexA].w = wA;
//            this.m_velocities[indexB].v = vB.Copy();
            this.m_velocities[indexB].w = wB;
        }
    }
    b2ContactSolver.prototype.FinalizeVelocityConstraints = function() {
        var i = 0;
        var j = 0;
        var point1, point2;
        for (i = 0; i < this.m_constraintCount; ++i) {
            var c = this.m_constraints[i];
            var m = c.manifold;
            for (j = 0; j < c.pointCount; ++j) {
                point1 = m.m_points[j];
                point2 = c.points[j];
                point1.normalImpulse = point2.normalImpulse;
                point1.tangentImpulse = point2.tangentImpulse;
            }
        }
    }
    b2ContactSolver.prototype.SolvePositionConstraints = function() {
        var minSeparation = 0.0;

        var i = 0;
        var j = 0;
        var xfA = b2ContactSolver.s_xfA, xfB = b2ContactSolver.s_xfB;
        for (i = 0; i < this.m_count; ++i) {
            var pc = this.m_positionConstraints[i];

            var indexA = pc.indexA;
            var indexB = pc.indexB;
            var localCenterA = pc.localCenterA;
            var mA = pc.invMassA;
            var iA = pc.invIA;
            var localCenterB = pc.localCenterB;
            var mB = pc.invMassB;
            var iB = pc.invIB;
            var pointCount = pc.pointCount;

            var cA = this.m_positions[indexA].c;
            var aA = this.m_positions[indexA].a;

            var cB = this.m_positions[indexB].c;
            var aB = this.m_positions[indexB].a;

            // Solve normal constraints
            var psm = b2ContactSolver.s_psm;
            for (j = 0; j < pointCount; ++j) {
                xfA.q.Set(aA);
                xfB.q.Set(aB);
                xfA.p.x = cA.x - (xfA.q.c * localCenterA.x - xfA.q.s * localCenterA.y);
                xfA.p.y = cA.y - (xfA.q.s * localCenterA.x + xfA.q.c * localCenterA.y);
                xfB.p.x = cB.x - (xfB.q.c * localCenterB.x - xfB.q.s * localCenterB.y);
                xfB.p.y = cB.y - (xfB.q.s * localCenterB.x + xfB.q.c * localCenterB.y);

                psm.Initialize(pc, xfA, xfB, j);
                var normal = psm.normal;
                var point = psm.point;
                var separation = psm.separation;

                var rAX = point.x - cA.x;
                var rAY = point.y - cA.y;
                var rBX = point.x - cB.x;
                var rBY = point.y - cB.y;

                // Track max constraint error.
                minSeparation = b2Math.Min(minSeparation, separation);

                // Prevent large corrections and allow slop.
                var C = b2Math.Clamp(b2Settings.b2_baumgarte * (separation + b2Settings.b2_linearSlop), -b2Settings.b2_maxLinearCorrection, 0.0);

                // Compute the effective mass.
                var rnA = rAX * normal.y - rAY * normal.x;
                var rnB = rBX * normal.y - rBY * normal.x;
                var K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

                // Compute normal impulse
                var impulse = K > 0.0 ? - C / K : 0.0;

                var PX = impulse * normal.x;
                var PY = impulse * normal.y;

                cA.x -= mA * PX;
                cA.y -= mA * PY;
                aA -= iA * (rAX * PY - rAY * PX);

                cB.x += mB * PX;
                cB.y += mB * PY;
                aB += iB * (rBX * PY - rBY * PX);
            }

//            this.m_positions[indexA].c = cA.Copy();
            this.m_positions[indexA].a = aA;
//            this.m_positions[indexB].c = cB.Copy();
            this.m_positions[indexB].a = aB;
        }

        // We can't expect minSpeparation >= -b2_linearSlop because we don't
        // push the separation above -b2_linearSlop.
        return minSeparation >= -3.0 * b2Settings.b2_linearSlop;
    }
    b2ContactSolver.prototype.StoreImpulses = function() {
        var i = 0;
        var j = 0;
        for (i = 0; i < this.m_count; ++i) {
            var vc = this.m_velocityConstraints[i];
            var manifold = this.m_contacts[vc.contactIndex].GetManifold();

            for (j = 0; j < vc.pointCount; ++j) {
                manifold.points[j].normalImpulse = vc.points[j].normalImpulse;
                manifold.points[j].tangentImpulse = vc.points[j].tangentImpulse;
            }
        }
    }
    b2ContactSolver.prototype.SolveTOIPositionConstraints = function(toiIndexA, toiIndexB) {
        var minSeparation = 0.0;

        var i = 0;
        var j = 0;
        for (i = 0; i < this.m_count; ++i) {
            var pc = this.m_positionConstraints[i];

            var indexA = pc.indexA;
            var indexB = pc.indexB;
            var localCenterA = pc.localCenterA;
            var localCenterB = pc.localCenterB;
            var pointCount = pc.pointCount;

            var mA = 0.0;
            var iA = 0.0;
            if (indexA == toiIndexA || indexA == toiIndexB) {
                mA = pc.invMassA;
                iA = pc.invIA;
            }

            var mB = pc.invMassB;
            var iB = pc.invIB;
            if (indexB == toiIndexA || indexB == toiIndexB) {
                mB = pc.invMassB;
                iB = pc.invIB;
            }

            var cA = this.m_positions[indexA].c;
            var aA = this.m_positions[indexA].a;
            var cB = this.m_positions[indexB].c;
            var aB = this.m_positions[indexB].a;

            // Solve normal constraints
            for (j = 0; j < pointCount; ++j) {
                var xfA = new b2Transform(), xfB = new b2Transform();
                xfA.q.Set(aA);
                xfB.q.Set(aB);
                xfA.p = b2Math.SubtractVV(cA, b2Math.MulRV(xfA.q, localCenterA));
                xfB.p = b2Math.SubtractVV(cB, b2Math.MulRV(xfB.q, localCenterB));

                var psm = b2ContactSolver.s_psm;
                psm.Initialize(pc, xfA, xfB, j);
                var normal = psm.normal;

                var point = psm.point;
                var separation = psm.separation;

                var rA = b2Math.SubtractVV(point, cA);
                var rB = b2Math.SubtractVV(point, cB);

                // Track max constraint error.
                minSeparation = b2Math.Min(minSeparation, separation);

                // Prevent large corrections and allow slop.
                var C = b2Math.Clamp(b2Settings.b2_toiBaugarte * (separation + b2Settings.b2_linearSlop), -b2_maxLinearCorrection, 0.0);

                // Compute the effective mass.
                var rnA = b2Math.CrossVV(rA, normal);
                var rnB = b2Math.CrossVV(rB, normal);
                var K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

                // Compute normal impulse
                var impulse = K > 0.0 ? - C / K : 0.0;

                var P = b2Math.MulFV(impulse, normal);

                cA.Subtract(b2Math.MulFV(mA, P));
                aA -= iA * b2Math.CrossVV(rA, P);

                cB.Add(b2Math.MulFV(mB, P));
                aB += iB * b2Math.CrossVV(rB, P);
            }

//            this.m_positions[indexA].c = cA.Copy();
            this.m_positions[indexA].a = aA;
//            this.m_positions[indexB].c = cB.Copy();
            this.m_positions[indexB].a = aB;
        }

        // We can't expect minSpeparation >= -b2_linearSlop because we don't
        // push the separation above -b2_linearSlop.
        return minSeparation >= -1.5 * b2Settings.b2_linearSlop;
    }
    Box2D.postDefs.push(function() {
        Box2D.Dynamics.Contacts.b2ContactSolver.s_xfA = new b2Transform();
        Box2D.Dynamics.Contacts.b2ContactSolver.s_xfB = new b2Transform();
        Box2D.Dynamics.Contacts.b2ContactSolver.s_b2WorldManifold = new b2WorldManifold();
        Box2D.Dynamics.Contacts.b2ContactSolver.s_psm = new b2PositionSolverManifold();
    });
    b2ContactSolverDef.b2ContactSolverDef = function() {
//        this.step = new b2TimeStep();
    };
    b2ContactSolverDef.prototype.b2ContactSolverDef = function() {
        this.contacts = null;
        this.count = 0;
        this.positions = null;
        this.step = null;
        this.velocities = null;
        this.allocator = null;
    }
    b2ContactVelocityConstraint.b2ContactVelocityConstraint = function() {
        this.normal = new b2Vec2();
        this.normalMass = new b2Mat22();
        this.K = new b2Mat22();
    };
    b2ContactVelocityConstraint.prototype.b2ContactVelocityConstraint = function() {
        this.points = new Vector(b2Settings.b2_maxManifoldPoints);
        for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
            this.points[i] = new b2VelocityConstraintPoint();
        }
    }
    Box2D.inherit(b2EdgeAndCircleContact, Box2D.Dynamics.Contacts.b2Contact);
    b2EdgeAndCircleContact.prototype.__super = Box2D.Dynamics.Contacts.b2Contact.prototype;
    b2EdgeAndCircleContact.b2EdgeAndCircleContact = function() {
        Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply(this, arguments);
    };
    b2EdgeAndCircleContact.Create = function(allocator) {
        return new b2EdgeAndCircleContact();
    }
    b2EdgeAndCircleContact.Destroy = function(contact, allocator) {}
    b2EdgeAndCircleContact.prototype.Reset = function(fixtureA, indexA, fixtureB, indexB) {
        this.__super.Reset.call(this, fixtureA, indexA, fixtureB, indexB);
//        b2Settings.b2Assert(this.m_fixtureA.GetType() === b2Shape.e_edge);
//        b2Settings.b2Assert(this.m_fixtureB.GetType() === b2Shape.e_circle);
    }
    b2EdgeAndCircleContact.prototype.Evaluate = function(manifold, xfA, xfB) {
        b2Collision.CollideEdgeAndCircle(manifold, (this.m_fixtureA.GetShape() instanceof b2EdgeShape ? this.m_fixtureA.GetShape() : null), xfA, (this.m_fixtureB.GetShape() instanceof b2CircleShape ? this.m_fixtureB.GetShape() : null), xfB);
    }

    Box2D.inherit(b2PolygonAndCircleContact, Box2D.Dynamics.Contacts.b2Contact);
    b2PolygonAndCircleContact.prototype.__super = Box2D.Dynamics.Contacts.b2Contact.prototype;
    b2PolygonAndCircleContact.b2PolygonAndCircleContact = function() {
        Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply(this, arguments);
    };
    b2PolygonAndCircleContact.Create = function(allocator) {
        return new b2PolygonAndCircleContact();
    }
    b2PolygonAndCircleContact.Destroy = function(contact, allocator) {}
    b2PolygonAndCircleContact.prototype.Reset = function(fixtureA, indexA, fixtureB, indexB) {
        this.__super.Reset.call(this, fixtureA, indexA, fixtureB, indexB);
//        b2Settings.b2Assert(this.m_fixtureA.GetType() === b2Shape.e_polygon);
//        b2Settings.b2Assert(this.m_fixtureB.GetType() === b2Shape.e_circle);
    }
    b2PolygonAndCircleContact.prototype.Evaluate = function(manifold, xfA, xfB) {
        b2Collision.CollidePolygonAndCircle(manifold, (this.m_fixtureA.GetShape() instanceof b2PolygonShape ? this.m_fixtureA.GetShape() : null), xfA, (this.m_fixtureB.GetShape() instanceof b2CircleShape ? this.m_fixtureB.GetShape() : null), xfB);
    }

    Box2D.inherit(b2EdgeAndPolygonContact, Box2D.Dynamics.Contacts.b2Contact);
    b2EdgeAndPolygonContact.prototype.__super = Box2D.Dynamics.Contacts.b2Contact.prototype;
    b2EdgeAndPolygonContact.b2EdgeAndPolygonContact = function() {
        Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply(this, arguments);
    };
    b2EdgeAndPolygonContact.Create = function(allocator) {
        return new b2EdgeAndPolygonContact();
    }
    b2EdgeAndPolygonContact.Destroy = function(contact, allocator) {}
    b2EdgeAndPolygonContact.prototype.Reset = function(fixtureA, indexA, fixtureB, indexB) {
        this.__super.Reset.call(this, fixtureA, indexA, fixtureB, indexB);
//        b2Settings.b2Assert(this.m_fixtureA.GetType() === b2Shape.e_edge);
//        b2Settings.b2Assert(this.m_fixtureB.GetType() === b2Shape.e_polygon);
    }
    b2EdgeAndPolygonContact.prototype.Evaluate = function(manifold, xfA, xfB) {
        b2Collision.CollideEdgeAndPolygon(manifold, (this.m_fixtureA.GetShape() instanceof b2EdgeShape ? this.m_fixtureA.GetShape() : null), xfA, (this.m_fixtureB.GetShape() instanceof b2PolygonShape ? this.m_fixtureB.GetShape() : null), xfB);
    }

    Box2D.inherit(b2PolygonContact, Box2D.Dynamics.Contacts.b2Contact);
    b2PolygonContact.prototype.__super = Box2D.Dynamics.Contacts.b2Contact.prototype;
    b2PolygonContact.b2PolygonContact = function() {
        Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply(this, arguments);
    };
    b2PolygonContact.Create = function(allocator) {
        return new b2PolygonContact();
    }
    b2PolygonContact.Destroy = function(contact, allocator) {}
    b2PolygonContact.prototype.Reset = function(fixtureA, indexA, fixtureB, indexB) {
        this.__super.Reset.call(this, fixtureA, indexA, fixtureB, indexB);
    }
    b2PolygonContact.prototype.Evaluate = function(manifold, xfA, xfB) {
        var shapeA = this.m_fixtureA.GetShape();
        if ( !(shapeA instanceof b2PolygonShape) )
            shapeA = null;
        var shapeB = this.m_fixtureB.GetShape();
        if ( !(shapeB instanceof b2PolygonShape) )
            shapeB = null;
        b2Collision.CollidePolygons(manifold, shapeA , xfA, shapeB, xfB);
    }

    b2PositionSolverManifold.b2PositionSolverManifold = function() {};
    b2PositionSolverManifold.prototype.b2PositionSolverManifold = function() {
        this.normal = new b2Vec2();
        this.separation = 0;
        this.point = new b2Vec2();
    }
    b2PositionSolverManifold.prototype.Initialize = function(pc, xfA, xfB, index) {
        b2Settings.b2Assert(pc.pointCount > 0);

        var tRot;
        var tVec;
        switch (pc.type) {
        case b2Manifold.e_circles:
        {
            tRot = xfA.q;
            tVec = pc.localPoint;
            var pointAX = tRot.c * tVec.x - tRot.s * tVec.y + xfA.p.x;
            var pointAY = tRot.s * tVec.x + tRot.c * tVec.y + xfA.p.y;
            tRot = xfB.q;
            tVec = pc.localPoints[0];
            var pointBX = tRot.c * tVec.x - tRot.s * tVec.y + xfB.p.x;
            var pointBY = tRot.s * tVec.x + tRot.c * tVec.y + xfB.p.y;
            this.normal.Set(pointBX - pointAX, pointBY - pointAY);
            this.normal.Normalize();
            this.point.Set(0.5 * (pointAX + pointBX), 0.5 * (pointAY + pointBY));
            this.separation = (pointBX - pointAX) * this.normal.x + (pointBY - pointAY) * this.normal.y - pc.radiusA - pc.radiusB;
        }
        break;

        case b2Manifold.e_faceA:
        {
            tRot = xfA.q;
            tVec = pc.localNormal;
            this.normal.x = tRot.c * tVec.x - tRot.s * tVec.y;
            this.normal.y = tRot.s * tVec.x + tRot.c * tVec.y;
            tVec = pc.localPoint;
            var planePointX = tRot.c * tVec.x - tRot.s * tVec.y + xfA.p.x;
            var planePointY = tRot.s * tVec.x + tRot.c * tVec.y + xfA.p.y;

            tRot = xfB.q;
            tVec = pc.localPoints[index];
            var clipPointX = tRot.c * tVec.x - tRot.s * tVec.y + xfB.p.x;
            var clipPointY = tRot.s * tVec.x + tRot.c * tVec.y + xfB.p.y;
            this.separation = (clipPointX - planePointX) * this.normal.x + (clipPointY - planePointY) * this.normal.y - pc.radiusA - pc.radiusB;
            this.point.Set(clipPointX, clipPointY);
        }
        break;

        case b2Manifold.e_faceB:
        {
            tRot = xfB.q;
            tVec = pc.localNormal;
            this.normal.x = tRot.c * tVec.x - tRot.s * tVec.y;
            this.normal.y = tRot.s * tVec.x + tRot.c * tVec.y;
            tVec = pc.localPoint;
            var planePointX = tRot.c * tVec.x - tRot.s * tVec.y + xfB.p.x;
            var planePointY = tRot.s * tVec.x + tRot.c * tVec.y + xfB.p.y;

            tRot = xfA.q;
            tVec = pc.localPoints[index];
            var clipPointX = tRot.c * tVec.x - tRot.s * tVec.y + xfA.p.x;
            var clipPointY = tRot.s * tVec.x + tRot.c * tVec.y + xfA.p.y;
            this.separation = (clipPointX - planePointX) * this.normal.x + (clipPointY - planePointY) * this.normal.y - pc.radiusA - pc.radiusB;
            this.point.Set(clipPointX, clipPointY);

            // Ensure normal points from A to B
            this.normal.x = -this.normal.x;
            this.normal.y = -this.normal.y;
        }
        break;
        }
    }
    b2VelocityConstraintPoint.b2VelocityConstraintPoint = function() {
        this.rA = new b2Vec2();
        this.rB = new b2Vec2();
    };
})();
(function() {
    var b2Body = Box2D.Dynamics.b2Body,
        b2BodyDef = Box2D.Dynamics.b2BodyDef,
        b2ContactFilter = Box2D.Dynamics.b2ContactFilter,
        b2ContactImpulse = Box2D.Dynamics.b2ContactImpulse,
        b2ContactListener = Box2D.Dynamics.b2ContactListener,
        b2ContactManager = Box2D.Dynamics.b2ContactManager,
        b2Draw = Box2D.Dynamics.b2Draw,
        b2DestructionListener = Box2D.Dynamics.b2DestructionListener,
        b2Filter = Box2D.Dynamics.b2Filter,
        b2Fixture = Box2D.Dynamics.b2Fixture,
        b2FixtureDef = Box2D.Dynamics.b2FixtureDef,
        b2Island = Box2D.Dynamics.b2Island,
        b2Position = Box2D.Dynamics.b2Position,
        b2Profile = Box2D.Dynamics.b2Profile,
        b2SolverData = Box2D.Dynamics.b2SolverData,
        b2TimeStep = Box2D.Dynamics.b2TimeStep,
        b2Velocity = Box2D.Dynamics.b2Velocity,
        b2World = Box2D.Dynamics.b2World,
        b2Mat22 = Box2D.Common.Math.b2Mat22,
        b2Mat33 = Box2D.Common.Math.b2Mat33,
        b2Rot = Box2D.Common.Math.b2Rot,
        b2Math = Box2D.Common.Math.b2Math,
        b2Sweep = Box2D.Common.Math.b2Sweep,
        b2Transform = Box2D.Common.Math.b2Transform,
        b2Vec2 = Box2D.Common.Math.b2Vec2,
        b2Vec3 = Box2D.Common.Math.b2Vec3,
        b2Color = Box2D.Common.b2Color,
        b2Settings = Box2D.Common.b2Settings,
        b2CircleShape = Box2D.Collision.Shapes.b2CircleShape,
        b2EdgeShape = Box2D.Collision.Shapes.b2EdgeShape,
        b2ChainShape = Box2D.Collision.Shapes.b2ChainShape,
        b2MassData = Box2D.Collision.Shapes.b2MassData,
        b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape,
        b2Shape = Box2D.Collision.Shapes.b2Shape;
})();
(function() {
    var b2Color = Box2D.Common.b2Color,
        b2Settings = Box2D.Common.b2Settings,
        b2Mat22 = Box2D.Common.Math.b2Mat22,
        b2Mat33 = Box2D.Common.Math.b2Mat33,
        b2Rot = Box2D.Common.Math.b2Rot,
        b2Math = Box2D.Common.Math.b2Math,
        b2Sweep = Box2D.Common.Math.b2Sweep,
        b2Transform = Box2D.Common.Math.b2Transform,
        b2Vec2 = Box2D.Common.Math.b2Vec2,
        b2Vec3 = Box2D.Common.Math.b2Vec3,
        b2DistanceJoint = Box2D.Dynamics.Joints.b2DistanceJoint,
        b2DistanceJointDef = Box2D.Dynamics.Joints.b2DistanceJointDef,
        b2FrictionJoint = Box2D.Dynamics.Joints.b2FrictionJoint,
        b2FrictionJointDef = Box2D.Dynamics.Joints.b2FrictionJointDef,
        b2GearJoint = Box2D.Dynamics.Joints.b2GearJoint,
        b2GearJointDef = Box2D.Dynamics.Joints.b2GearJointDef,
        b2Jacobian = Box2D.Dynamics.Joints.b2Jacobian,
        b2Joint = Box2D.Dynamics.Joints.b2Joint,
        b2JointDef = Box2D.Dynamics.Joints.b2JointDef,
        b2JointEdge = Box2D.Dynamics.Joints.b2JointEdge,
        b2MouseJoint = Box2D.Dynamics.Joints.b2MouseJoint,
        b2MouseJointDef = Box2D.Dynamics.Joints.b2MouseJointDef,
        b2PrismaticJoint = Box2D.Dynamics.Joints.b2PrismaticJoint,
        b2PrismaticJointDef = Box2D.Dynamics.Joints.b2PrismaticJointDef,
        b2PulleyJoint = Box2D.Dynamics.Joints.b2PulleyJoint,
        b2PulleyJointDef = Box2D.Dynamics.Joints.b2PulleyJointDef,
        b2RevoluteJoint = Box2D.Dynamics.Joints.b2RevoluteJoint,
        b2RevoluteJointDef = Box2D.Dynamics.Joints.b2RevoluteJointDef,
        b2RopeJoint = Box2D.Dynamics.Joints.b2RopeJoint,
        b2RopeJointDef = Box2D.Dynamics.Joints.b2RopeJointDef,
        b2WeldJoint = Box2D.Dynamics.Joints.b2WeldJoint,
        b2WeldJointDef = Box2D.Dynamics.Joints.b2WeldJointDef,
        b2WheelJoint = Box2D.Dynamics.Joints.b2WheelJoint,
        b2WheelJointDef = Box2D.Dynamics.Joints.b2WheelJointDef,
        b2Body = Box2D.Dynamics.b2Body,
        b2BodyDef = Box2D.Dynamics.b2BodyDef,
        b2ContactFilter = Box2D.Dynamics.b2ContactFilter,
        b2ContactImpulse = Box2D.Dynamics.b2ContactImpulse,
        b2ContactListener = Box2D.Dynamics.b2ContactListener,
        b2ContactManager = Box2D.Dynamics.b2ContactManager,
        b2Draw = Box2D.Dynamics.b2Draw,
        b2DestructionListener = Box2D.Dynamics.b2DestructionListener,
        b2Filter = Box2D.Dynamics.b2Filter,
        b2Fixture = Box2D.Dynamics.b2Fixture,
        b2FixtureDef = Box2D.Dynamics.b2FixtureDef,
        b2Island = Box2D.Dynamics.b2Island,
        b2Position = Box2D.Dynamics.b2Position,
        b2Profile = Box2D.Dynamics.b2Profile,
        b2SolverData = Box2D.Dynamics.b2SolverData,
        b2TimeStep = Box2D.Dynamics.b2TimeStep,
        b2Velocity = Box2D.Dynamics.b2Velocity,
        b2World = Box2D.Dynamics.b2World;

    Box2D.inherit(b2DistanceJoint, Box2D.Dynamics.Joints.b2Joint);
    b2DistanceJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
    b2DistanceJoint.b2DistanceJoint = function() {
        Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
        this.m_localAnchorA = new b2Vec2();
        this.m_localAnchorB = new b2Vec2();
        this.m_u = new b2Vec2();
        this.m_rA = new b2Vec2();
        this.m_rB = new b2Vec2();
        this.m_localCenterA = new b2Vec2();
        this.m_localCenterB = new b2Vec2();
    };
    b2DistanceJoint.prototype.GetLocalAnchorA = function() {
        return this.m_localAnchorA;
    }
    b2DistanceJoint.prototype.GetLocalAnchorB = function() {
        return this.m_localAnchorB;
    }
    b2DistanceJoint.prototype.GetAnchorA = function() {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
    }
    b2DistanceJoint.prototype.GetAnchorB = function() {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
    }
    b2DistanceJoint.prototype.GetReactionForce = function(inv_dt) {
        return new b2Vec2(inv_dt * this.m_impulse * this.m_u.x, inv_dt * this.m_impulse * this.m_u.y);
    }
    b2DistanceJoint.prototype.GetReactionTorque = function(inv_dt) {
        return 0.0;
    }
    b2DistanceJoint.prototype.GetLength = function() {
        return this.m_length;
    }
    b2DistanceJoint.prototype.SetLength = function(length) {
        this.m_length = length;
    }
    b2DistanceJoint.prototype.GetFrequency = function() {
        return this.m_frequencyHz;
    }
    b2DistanceJoint.prototype.SetFrequency = function(hz) {
        this.m_frequencyHz = hz;
    }
    b2DistanceJoint.prototype.GetDampingRatio = function() {
        return this.m_dampingRatio;
    }
    b2DistanceJoint.prototype.SetDampingRatio = function(ratio) {
        this.m_dampingRatio = ratio;
    }
    b2DistanceJoint.prototype.b2DistanceJoint = function(def) {
        this.__super.b2Joint.call(this, def);
        this.m_localAnchorA.SetV(def.localAnchorA);
        this.m_localAnchorB.SetV(def.localAnchorB);
        this.m_length = def.length;
        this.m_frequencyHz = def.frequencyHz;
        this.m_dampingRatio = def.dampingRatio;
        this.m_impulse = 0.0;
        this.m_gamma = 0.0;
        this.m_bias = 0.0;
    }
    b2DistanceJoint.prototype.InitVelocityConstraints = function(data) {
        this.m_indexA = this.m_bodyA.m_islandIndex;
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_localCenterA.SetV(this.m_bodyA.m_sweep.localCenter);
        this.m_localCenterB.SetV(this.m_bodyB.m_sweep.localCenter);
        this.m_invMassA = this.m_bodyA.m_invMass;
        this.m_invMassB = this.m_bodyB.m_invMass;
        this.m_invIA = this.m_bodyA.m_invI;
        this.m_invIB = this.m_bodyB.m_invI;

        var cA = data.positions[this.m_indexA].c;
        var aA = data.positions[this.m_indexA].a;
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;

        var cB = data.positions[this.m_indexB].c;
        var aB = data.positions[this.m_indexB].a;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;

        var qA = new b2Rot(aA), qB = new b2Rot(aB);

        this.m_rA = b2Math.MulRV(qA, b2Math.SubtractVV(this.m_localAnchorA, this.m_localCenterA));
        this.m_rB = b2Math.MulRV(qB, b2Math.SubtractVV(this.m_localAnchorB, this.m_localCenterB));
        this.m_u = b2Math.AddVV(cB, this.m_rB);
        this.m_u.Subtract(cA); this.m_u.Subtract(this.m_rA);

        // Handle singularity.
        var length = this.m_u.Length();
        if (length > b2Settings.b2_linearSlop) {
            this.m_u.Multiply(1.0 / length);
        } else {
            this.m_u.Set(0.0, 0.0);
        }

        var crAu = b2Math.CrossVV(this.m_rA, this.m_u);
        var crBu = b2Math.CrossVV(this.m_rB, this.m_u);
        var invMass = this.m_invMassA + this.m_invIA * crAu * crAu + this.m_invMassB + this.m_invIB * crBu * crBu;

        // Compute the effective mass matrix.
        this.m_mass = invMass != 0.0 ? 1.0 / invMass : 0.0;

        if (this.m_frequencyHz > 0.0) {
            var C = length - this.m_length;

            // Frequency
            var omega = 2.0 * b2Settings.b2_pi * this.m_frequencyHz;

            // Damping coefficient
            var d = 2.0 * this.m_mass * this.m_dampingRatio * omega;

            // Spring stiffness
            var k = this.m_mass * omega * omega;

            // magic formulas
            var h = data.step.dt;
            this.m_gamma = h * (d + h * k);
            this.m_gamma = this.m_gamma != 0.0 ? 1.0 / this.m_gamma : 0.0;
            this.m_bias = C * h * k * this.m_gamma;

            invMass += this.m_gamma;
            this.m_mass = invMass != 0.0 ? 1.0 / invMass : 0.0;
        } else {
            this.m_gamma = 0.0;
            this.m_bias = 0.0;
        }

        if (data.step.warmStarting) {
            // Scale the impulse to support a variable time step.
            this.m_impulse *= data.step.dtRatio;

            var P = b2Math.MulFV(this.m_impulse, this.m_u);
            vA.Subtract(b2Math.MulFV(this.m_invMassA, P));
            wA -= this.m_invIA * b2Math.CrossVV(this.m_rA, P);
            vB.Add(b2Math.MulFV(this.m_invMassB, P));
            wB += this.m_invIB * b2Math.CrossVV(this.m_rB, P);
        } else {
            this.m_impulse = 0.0;
        }

//        data.velocities[this.m_indexA].v = vA.Copy();
        data.velocities[this.m_indexA].w = wA;
//        data.velocities[this.m_indexB].v = vB.Copy();
        data.velocities[this.m_indexB].w = wB;
    }
    b2DistanceJoint.prototype.SolveVelocityConstraints = function(data) {
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;

        var vpA = b2Math.AddVV(vA, b2Math.CrossFV(wA, this.m_rA));
        var vpB = b2Math.AddVV(vB, b2Math.CrossFV(wB, this.m_rB));
        var Cdot = b2Math.DotVV(this.m_u, b2Math.SubtractVV(vpB, vpA));

        var impulse = -this.m_mass * (Cdot + this.m_bias + this.m_gamma * this.m_impulse);
        this.m_impulse += impulse;

        var P = b2Math.MulFV(impulse, this.m_u);
        vA.Subtract(b2Math.MulFV(this.m_invMassA, P));
        wA -= this.m_invIA * b2Math.CrossVV(this.m_rA, P);
        vB.Add(b2Math.MulFV(this.m_invMassB, P));
        wB += this.m_invIB * b2Math.CrossVV(this.m_rB, P);

//        data.velocities[this.m_indexA].v = vA.Copy();
        data.velocities[this.m_indexA].w = wA;
//        data.velocities[this.m_indexB].v = vB.Copy();
        data.velocities[this.m_indexB].w = wB;
    }
    b2DistanceJoint.prototype.SolvePositionConstraints = function(data) {
        if (this.m_frequencyHz > 0.0) {
            return true;
        }
        var cA = data.positions[this.m_indexA].c;
        var aA = data.positions[this.m_indexA].a;
        var cB = data.positions[this.m_indexB].c;
        var aB = data.positions[this.m_indexB].a;

        var qA = new b2Rot(aA), qB = new b2Rot(aB);

        var rA = b2Math.MulRV(qA, b2Math.SubtractVV(this.m_localAnchorA, this.m_localCenterA));
        var rB = b2Math.MulRV(qB, b2Math.SubtractVV(this.m_localAnchorB, this.m_localCenterB));
        var u = b2Math.AddVV(cB, rB); u.Subtract(cA); u.Subtract(rA);

        var length = u.Normalize();
        var C = length - this.m_length;
        C = b2Math.Clamp(C, -b2Settings.b2_maxLinearCorrection, b2Settings.b2_maxLinearCorrection);

        var impulse = -this.m_mass * C;
        var P = b2Math.MulFV(impulse, u);

        cA.Subtract(b2Math.MulFV(this.m_invMassA, P));
        aA -= this.m_invIA * b2Math.CrossVV(rA, P);
        cB.Add(b2Math.MulFV(this.m_invMassB, P));
        aB += this.m_invIB * b2Math.CrossVV(rB, P);

//        data.positions[this.m_indexA].c = cA.Copy();
        data.positions[this.m_indexA].a = aA;
//        data.positions[this.m_indexB].c = cB.Copy();
        data.positions[this.m_indexB].a = aB;

        return b2Math.Abs(C) < b2Settings.b2_linearSlop;
    }
    Box2D.inherit(b2DistanceJointDef, Box2D.Dynamics.Joints.b2JointDef);
    b2DistanceJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
    b2DistanceJointDef.b2DistanceJointDef = function() {
        Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
        this.localAnchorA = new b2Vec2();
        this.localAnchorB = new b2Vec2();
    };
    b2DistanceJointDef.prototype.b2DistanceJointDef = function() {
        this.__super.b2JointDef.call(this);
        this.type = b2Joint.e_distanceJoint;
        this.length = 1.0;
        this.frequencyHz = 0.0;
        this.dampingRatio = 0.0;
    }
    b2DistanceJointDef.prototype.Initialize = function(bA, bB, anchorA, anchorB) {
        this.bodyA = bA;
        this.bodyB = bB;
        this.localAnchorA.SetV(this.bodyA.GetLocalPoint(anchorA));
        this.localAnchorB.SetV(this.bodyB.GetLocalPoint(anchorB));
        var dX = anchorB.x - anchorA.x;
        var dY = anchorB.y - anchorA.y;
        this.length = Math.sqrt(dX * dX + dY * dY);
    }
    Box2D.inherit(b2FrictionJoint, Box2D.Dynamics.Joints.b2Joint);
    b2FrictionJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
    b2FrictionJoint.b2FrictionJoint = function() {
        Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
        this.m_localAnchorA = new b2Vec2();
        this.m_localAnchorB = new b2Vec2();
        this.m_linearMass = new b2Mat22();
        this.m_linearImpulse = new b2Vec2();
        this.m_rA = new b2Vec2();
        this.m_rB = new b2Vec2();
        this.m_localCenterA = new b2Vec2();
        this.m_localCenterB = new b2Vec2();
    };
    b2FrictionJoint.prototype.GetLocalAnchorA = function() {
        return this.m_localAnchorA;
    }
    b2FrictionJoint.prototype.GetLocalAnchorB = function() {
        return this.m_localAnchorB;
    }
    b2FrictionJoint.prototype.GetAnchorA = function() {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
    }
    b2FrictionJoint.prototype.GetAnchorB = function() {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
    }
    b2FrictionJoint.prototype.GetReactionForce = function(inv_dt) {
        return new b2Vec2(inv_dt * this.m_linearImpulse.x, inv_dt * this.m_linearImpulse.y);
    }
    b2FrictionJoint.prototype.GetReactionTorque = function(inv_dt) {
        return inv_dt * this.m_angularImpulse;
    }
    b2FrictionJoint.prototype.SetMaxForce = function(force) {
        this.m_maxForce = force;
    }
    b2FrictionJoint.prototype.GetMaxForce = function() {
        return this.m_maxForce;
    }
    b2FrictionJoint.prototype.SetMaxTorque = function(torque) {
        this.m_maxTorque = torque;
    }
    b2FrictionJoint.prototype.GetMaxTorque = function() {
        return this.m_maxTorque;
    }
    b2FrictionJoint.prototype.b2FrictionJoint = function(def) {
        this.__super.b2Joint.call(this, def);
        this.m_localAnchorA.SetV(def.localAnchorA);
        this.m_localAnchorB.SetV(def.localAnchorB);
        this.m_linearImpulse.SetZero();
        this.m_angularImpulse = 0.0;
        this.m_maxForce = def.maxForce;
        this.m_maxTorque = def.maxTorque;
    }
    b2FrictionJoint.prototype.InitVelocityConstraints = function(data) {
        this.m_indexA = this.m_bodyA.m_islandIndex;
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_localCenterA.SetV(this.m_bodyA.m_sweep.localCenter);
        this.m_localCenterB.SetV(this.m_bodyB.m_sweep.localCenter);
        this.m_invMassA = this.m_bodyA.m_invMass;
        this.m_invMassB = this.m_bodyB.m_invMass;
        this.m_invIA = this.m_bodyA.m_invI;
        this.m_invIB = this.m_bodyB.m_invI;

        var aA = data.positions[this.m_indexA].a;
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;

        var aB = data.positions[this.m_indexB].a;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;

        var qA = new b2Rot(aA), qB = new b2Rot(aB);

        // Compute the effective mass matrix.
        this.m_rA = b2Math.MulRV(qA, b2Math.SubtractVV(this.m_localAnchorA, this.m_localCenterA));
        this.m_rB = b2Math.MulRV(qB, b2Math.SubtractVV(this.m_localAnchorB, this.m_localCenterB));

        // J = [-I -r1_skew I r2_skew]
        //     [ 0       -1 0       1]
        // r_skew = [-ry; rx]

        // Matlab
        // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
        //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
        //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

        var mA = this.m_invMassA, mB = this.m_invMassB;
        var iA = this.m_invIA, iB = this.m_invIB;

        var K = new b2Mat22();
        K.ex.x = mA + mB + iA * this.m_rA.y * this.m_rA.y + iB * this.m_rB.y * this.m_rB.y;
        K.ex.y = -iA * this.m_rA.x * this.m_rA.y - iB * this.m_rB.x * this.m_rB.y;
        K.ey.x = K.ex.y;
        K.ey.y = mA + mB + iA * this.m_rA.x * this.m_rA.x + iB * this.m_rB.x * this.m_rB.x;

        this.m_linearMass = K.GetInverse();

        this.m_angularMass = iA + iB;
        if (this.m_angularMass > 0.0) {
            this.m_angularMass = 1.0 / this.m_angularMass;
        }

        if (data.step.warmStarting) {
            // Scale impulses to support a variable time step.
            this.m_linearImpulse.Multiply(data.step.dtRatio);
            this.m_angularImpulse *= data.step.dtRatio;

            var P = new b2Vec2(this.m_linearImpulse.x, this.m_linearImpulse.y);
            vA.Subtract(b2Math.MulFV(mA, P));
            wA -= iA * (b2Math.CrossVV(this.m_rA, P) + this.m_angularImpulse);
            vB.Add(b2Math.MulFV(mB, P));
            wB += iB * (b2Math.CrossVV(this.m_rB, P) + this.m_angularImpulse);
        } else {
            this.m_linearImpulse.SetZero();
            this.m_angularImpulse = 0.0;
        }

//        data.velocities[this.m_indexA].v = vA.Copy();
        data.velocities[this.m_indexA].w = wA;
//        data.velocities[this.m_indexB].v = vB.Copy();
        data.velocities[this.m_indexB].w = wB;
    }
    b2FrictionJoint.prototype.SolveVelocityConstraints = function(data) {
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;

        var mA = this.m_invMassA, mB = this.m_invMassB;
        var iA = this.m_invIA, iB = this.m_invIB;

        var h = data.step.dt;

        // Solve angular friction
        {
            var Cdot = wB - wA;
            var impulse = -this.m_angularMass * Cdot;

            var oldImpulse = this.m_angularImpulse;
            var maxImpulse = h * this.m_maxTorque;
            this.m_angularImpulse = b2Math.Clamp(this.m_angularImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = this.m_angularImpulse - oldImpulse;

            wA -= iA * impulse;
            wB += iB * impulse;
        }

        // Solve linear friction
        {
            var Cdot = b2Math.AddVV(vB, b2Math.CrossSV(wB, this.m_rB))
            Cdot.Subtract(b2Math.AddVV(vA, b2Math.CrossSV(wA, this.m_rA)));

            var impulse = b2Math.MulMV(this.m_linearMass, Cdot).GetNegative();
            var oldImpulse = this.m_linearImpulse.Copy();
            this.m_linearImpulse.Add(impulse);

            var maxImpulse = h * this.m_maxForce;

            if (this.m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse) {
                this.m_linearImpulse.Normalize();
                this.m_linearImpulse.Multiply(maxImpulse);
            }

            impulse = b2Math.SubtractVV(this.m_linearImpulse, oldImpulse);

            vA.Subtract(b2Math.MulFV(mA, impulse));
            wA -= iA * b2Math.CrossVV(this.m_rA, impulse);

            vB.Add(b2Math.MulFV(mB, impulse));
            wB += iB * b2Math.CrossVV(this.m_rB, impulse);
        }

//        data.velocities[this.m_indexA].v = vA.Copy();
        data.velocities[this.m_indexA].w = wA;
//        data.velocities[this.m_indexB].v = vB.Copy();
        data.velocities[this.m_indexB].w = wB;
    }
    b2FrictionJoint.prototype.SolvePositionConstraints = function(baumgarte) {
        return true;
    }
    Box2D.inherit(b2FrictionJointDef, Box2D.Dynamics.Joints.b2JointDef);
    b2FrictionJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
    b2FrictionJointDef.b2FrictionJointDef = function() {
        Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
        this.localAnchorA = new b2Vec2();
        this.localAnchorB = new b2Vec2();
    };
    b2FrictionJointDef.prototype.b2FrictionJointDef = function() {
        this.__super.b2JointDef.call(this);
        this.type = b2Joint.e_frictionJoint;
        this.maxForce = 0.0;
        this.maxTorque = 0.0;
    }
    b2FrictionJointDef.prototype.Initialize = function(bA, bB, anchor) {
        this.bodyA = bA;
        this.bodyB = bB;
        this.localAnchorA.SetV(this.bodyA.GetLocalPoint(anchor));
        this.localAnchorB.SetV(this.bodyB.GetLocalPoint(anchor));
    }
    Box2D.inherit(b2GearJoint, Box2D.Dynamics.Joints.b2Joint);
    b2GearJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
    b2GearJoint.b2GearJoint = function() {
        Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
        this.m_localAnchorA = new b2Vec2();
        this.m_localAnchorB = new b2Vec2();
        this.m_localAnchorC = new b2Vec2();
        this.m_localAnchorD = new b2Vec2();

        this.m_localAxisC = new b2Vec2();
        this.m_localAxisD = new b2Vec2();

        this.m_lcA = new b2Vec2();
        this.m_lcB = new b2Vec2();
        this.m_lcC = new b2Vec2();
        this.m_lcD = new b2Vec2();
        this.m_JvAC = new b2Vec2();
        this.m_JvBD = new b2Vec2();
    };
    b2GearJoint.prototype.GetJoint1 = function() {
        return this.m_joint1;
    }
    b2GearJoint.prototype.GetJoint2 = function() {
        return this.m_joint2;
    }
    b2GearJoint.prototype.GetAnchorA = function() {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
    }
    b2GearJoint.prototype.GetAnchorB = function() {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
    }
    b2GearJoint.prototype.GetReactionForce = function(inv_dt) {
        return new b2Vec2(inv_dt * this.m_impulse * this.m_JvAC.x, inv_dt * this.m_impulse * this.m_JvAC.y);
    }
    b2GearJoint.prototype.GetReactionTorque = function(inv_dt) {
        return inv_dt * this.m_impulse * this.m_JwA;
    }
    b2GearJoint.prototype.GetRatio = function() {
        return this.m_ratio;
    }
    b2GearJoint.prototype.SetRatio = function(ratio) {
        this.m_ratio = ratio;
    }
    b2GearJoint.prototype.b2GearJoint = function(def) {
        this.__super.b2Joint.call(this, def);
        this.m_typeA = parseInt(def.joint1.m_type);
        this.m_typeB = parseInt(def.joint2.m_type);
        this.m_joint1 = def.joint1;
        this.m_joint2 = def.joint2;

        b2Settings.b2Assert(this.m_typeA == b2Joint.e_revoluteJoint || this.m_typeA == b2Joint.e_prismaticJoint);
        b2Settings.b2Assert(this.m_typeB == b2Joint.e_revoluteJoint || this.m_typeB == b2Joint.e_prismaticJoint);

        var coordinateA = 0;
        var coordinateB = 0;
        this.m_bodyC = def.joint1.GetBodyA();
        this.m_bodyA = def.joint1.GetBodyB();

        // Get geometry of joint1
        var xfA = this.m_bodyA.m_xf;
        var aA = this.m_bodyA.m_sweep.a;
        var xfC = this.m_bodyC.m_xf;
        var aC = this.m_bodyC.m_sweep.a;

        if (this.m_typeA == b2Joint.e_revoluteJoint) {
            var revolute = def.joint1;
            this.m_localAnchorC.SetV(revolute.m_localAnchorA);
            this.m_localAnchorA.SetV(revolute.m_localAnchorB);
            this.m_referenceAngleA = revolute.m_referenceAngle;
            this.m_localAxisC.SetZero();
            coordinateA = aA - aC - this.m_referenceAngleA;
        } else {
            var prismatic = def.joint1;
            this.m_localAnchorC.SetV(prismatic.m_localAnchorA);
            this.m_localAnchorA.SetV(prismatic.m_localAnchorB);
            this.m_referenceAngleA = prismatic.m_referenceAngle;
            this.m_localAxisC.SetV(prismatic.m_localXAxisA);

            var pC = this.m_localAnchorC;
            var pA = b2Math.MulTRV(xfC.q, b2Math.AddVV(b2Math.MulRV(xfA.q, this.m_localAnchorA), b2Math.SubtractVV(xfA.p, xfC.p)));
            coordinateA = b2Math.DotVV(b2Math.SubtractVV(pA, pC), this.m_localAxisC);
        }
        this.m_bodyD = def.joint2.GetBodyA();
        this.m_bodyB = def.joint2.GetBodyB();

        // Get geometry of joint2
        var xfB = this.m_bodyB.m_xf;
        var aB = this.m_bodyB.m_sweep.a;
        var xfD = this.m_bodyD.m_xf;
        var aD = this.m_bodyD.m_sweep.a;

        if (this.m_typeB == b2Joint.e_revoluteJoint) {
            this.m_localAnchorD.SetV(this.m_joint2.m_localAnchorA);
            this.m_localAnchorB.SetV(this.m_joint2.m_localAnchorB);
            this.m_referenceAngleB = this.m_joint2.m_referenceAngle;
            this.m_localAxisD.SetZero();
            coordinateB = aB - aD - this.m_referenceAngleB;
        } else {
            this.m_localAnchorD.SetV(this.m_joint2.m_localAnchorA);
            this.m_localAnchorB.SetV(this.m_joint2.m_localAnchorB);
            this.m_referenceAngleB = this.m_joint2.m_referenceAngle;
            this.m_localAxisD.SetV(this.m_joint2.m_localXAxisA);

            var pD = this.m_localAnchorD;
            var pB = b2Math.MulTRV(xfD.q, b2Math.AddVV(b2Math.MulRV(xfB.q, this.m_localAnchorB), b2Math.SubtractVV(xfB.p, xfD.p)));
            coordinateB = b2Math.DotVV(b2Math.SubtractVV(pB, pD), this.m_localAxisD);
        }
        this.m_ratio = def.ratio;
        this.m_constant = coordinateA + this.m_ratio * coordinateB;
        this.m_impulse = 0.0;
    }
    b2GearJoint.prototype.InitVelocityConstraints = function(data) {
        this.m_indexA = this.m_bodyA.m_islandIndex;
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_indexC = this.m_bodyC.m_islandIndex;
        this.m_indexD = this.m_bodyD.m_islandIndex;
        this.m_lcA.SetV(this.m_bodyA.m_sweep.localCenter);
        this.m_lcB.SetV(this.m_bodyB.m_sweep.localCenter);
        this.m_lcC.SetV(this.m_bodyC.m_sweep.localCenter);
        this.m_lcD.SetV(this.m_bodyD.m_sweep.localCenter);
        this.m_mA = this.m_bodyA.m_invMass;
        this.m_mB = this.m_bodyB.m_invMass;
        this.m_mC = this.m_bodyC.m_invMass;
        this.m_mD = this.m_bodyD.m_invMass;
        this.m_iA = this.m_bodyA.m_invI;
        this.m_iB = this.m_bodyB.m_invI;
        this.m_iC = this.m_bodyC.m_invI;
        this.m_iD = this.m_bodyD.m_invI;

        var cA = data.positions[this.m_indexA].c;
        var aA = data.positions[this.m_indexA].a;
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;

        var cB = data.positions[this.m_indexB].c;
        var aB = data.positions[this.m_indexB].a;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;

        var cC = data.positions[this.m_indexC].c;
        var aC = data.positions[this.m_indexC].a;
        var vC = data.velocities[this.m_indexC].v;
        var wC = data.velocities[this.m_indexC].w;

        var cD = data.positions[this.m_indexD].c;
        var aD = data.positions[this.m_indexD].a;
        var vD = data.velocities[this.m_indexD].v;
        var wD = data.velocities[this.m_indexD].w;

        var qA = new b2Rot(aA), qB = new b2Rot(aB), qC = new b2Rot(aC), qD = new b2Rot(aD);

        this.m_mass = 0.0;

        if (this.m_typeA == b2Joint.e_revoluteJoint) {
            this.m_JvAC.SetZero();
            this.m_JwA = 1.0;
            this.m_JwC = 1.0;
            this.m_mass += this.m_iA + this.m_iC;
        } else {
            var u = b2Math.MulRV(qC, this.m_localAxisC);
            var rC = b2Math.MulRV(qC, b2Math.SubtractVV(this.m_localAnchorC, this.m_lcC));
            var rA = b2Math.MulRV(qA, b2Math.SubtractVV(this.m_localAnchorA, this.m_lcA));
            this.m_JvAC = u;
            this.m_JwC = b2Math.CrossVV(rC, u);
            this.m_JwA = b2Math.CrossVV(rA, u);
            this.m_mass += this.m_mC + this.m_mA + this.m_iC * this.m_JwC * this.m_JwC + this.m_iA * this.m_JwA * this.m_JwA;
        }

        if (this.m_typeB == b2Joint.e_revoluteJoint) {
            this.m_JvBD.SetZero();
            this.m_JwB = this.m_ratio;
            this.m_JwD = this.m_ratio;
            this.m_mass += this.m_ratio * this.m_ratio * (this.m_iB + this.m_iD);
        } else {
            var u = b2Math.MulRV(qD, this.m_localAxisD);
            var rD = b2Math.MulRV(qD, b2Math.SubtractVV(this.m_localAnchorD, this.m_lcD));
            var rB = b2Math.MulRV(qB, b2Math.SubtractVV(this.m_localAnchorB, this.m_lcB));
            this.m_JvBD = b2Math.MulFV(this.m_ratio, u);
            this.m_JwD = this.m_ratio * b2Math.CrossVV(rD, u);
            this.m_JwB = this.m_ratio * b2Math.CrossVV(rB, u);
            this.m_mass += this.m_ratio * this.m_ratio * (this.m_mD + this.m_mB) + this.m_iD * this.m_JwD * this.m_JwD + this.m_iB * this.m_JwB * this.m_JwB;
        }

        // Compute effective mass.
        this.m_mass = this.m_mass > 0.0 ? 1.0 / this.m_mass : 0.0;

        if (data.step.warmStarting) {
            vA.Add(b2Math.MulFV(this.m_mA * this.m_impulse, this.m_JvAC));
            wA += this.m_iA * this.m_impulse * this.m_JwA;
            vB.Add(b2Math.MulFV(this.m_mB * this.m_impulse, this.m_JvBD));
            wB += this.m_iB * this.m_impulse * this.m_JwB;
            vC.Subtract(b2Math.MulFV(this.m_mC * this.m_impulse, this.m_JvAC));
            wC -= this.m_iC * this.m_impulse * this.m_JwC;
            vD.Subtract(b2Math.MulFV(this.m_mD * this.m_impulse, this.m_JvBD));
            wD -= this.m_iD * this.m_impulse * this.m_JwD;
        } else {
            this.m_impulse = 0.0;
        }

//        data.velocities[this.m_indexA].v = vA.Copy();
        data.velocities[this.m_indexA].w = wA;
//        data.velocities[this.m_indexB].v = vB.Copy();
        data.velocities[this.m_indexB].w = wB;
//        data.velocities[this.m_indexC].v = vC.Copy();
        data.velocities[this.m_indexC].w = wC;
//        data.velocities[this.m_indexD].v = vD.Copy();
        data.velocities[this.m_indexD].w = wD;
    }
    b2GearJoint.prototype.SolveVelocityConstraints = function(data) {
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;
        var vC = data.velocities[this.m_indexC].v;
        var wC = data.velocities[this.m_indexC].w;
        var vD = data.velocities[this.m_indexD].v;
        var wD = data.velocities[this.m_indexD].w;

        var Cdot = b2Math.DotVV(this.m_JvAC, b2Math.SubtractVV(vA, vC)) + b2Math.DotVV(this.m_JvBD, b2Math.SubtractVV(vB, vD));
        Cdot += (this.m_JwA * wA - this.m_JwC * wC) + (this.m_JwB * wB - this.m_JwD * wD);

        var impulse = -this.m_mass * Cdot;
        this.m_impulse += impulse;

        vA.Add(b2Math.MulFV(this.m_mA * impulse, this.m_JvAC));
        wA += this.m_iA * impulse * this.m_JwA;
        vB.Add(b2Math.MulFV(this.m_mB * impulse, this.m_JvBD));
        wB += this.m_iB * impulse * this.m_JwB;
        vC.Subtract(b2Math.MulFV(this.m_mC * impulse, this.m_JvAC));
        wC -= this.m_iC * impulse * this.m_JwC;
        vD.Subtract(b2Math.MulFV(this.m_mD * impulse, this.m_JvBD));
        wD -= this.m_iD * impulse * this.m_JwD;

//        data.velocities[this.m_indexA].v = vA.Copy();
        data.velocities[this.m_indexA].w = wA;
//        data.velocities[this.m_indexB].v = vB.Copy();
        data.velocities[this.m_indexB].w = wB;
//        data.velocities[this.m_indexC].v = vC.Copy();
        data.velocities[this.m_indexC].w = wC;
//        data.velocities[this.m_indexD].v = vD.Copy();
        data.velocities[this.m_indexD].w = wD;
    }
    b2GearJoint.prototype.SolvePositionConstraints = function(data) {
        var cA = data.positions[this.m_indexA].c;
        var aA = data.positions[this.m_indexA].a;
        var cB = data.positions[this.m_indexB].c;
        var aB = data.positions[this.m_indexB].a;
        var cC = data.positions[this.m_indexC].c;
        var aC = data.positions[this.m_indexC].a;
        var cD = data.positions[this.m_indexD].c;
        var aD = data.positions[this.m_indexD].a;

        var qA = new b2Rot(aA), qB = new b2Rot(aB), qC = new b2Rot(aC), qD = new b2Rot(aD);

        var linearError = 0.0;

        var coordinateA, coordinateB;

        var JvAC = new b2Vec2(), JvBD = new b2Vec2();
        var JwA, JwB, JwC, JwD;
        var mass = 0.0;

        if (this.m_typeA == b2Joint.e_revoluteJoint) {
            JvAC.SetZero();
            JwA = 1.0;
            JwC = 1.0;
            mass += this.m_iA + this.m_iC;

            coordinateA = aA - aC - this.m_referenceAngleA;
        } else {
            var u = b2Math.MulRV(qC, this.m_localAxisC);
            var rC = b2Math.MulRV(qC, b2Math.SubtractVV(this.m_localAnchorC, this.m_lcC));
            var rA = b2Math.MulRV(qA, b2Math.SubtractVV(this.m_localAnchorA, this.m_lcA));
            JvAC = u;
            JwC = b2Math.CrossVV(rC, u);
            JwA = b2Math.CrossVV(rA, u);
            mass += this.m_mC + this.m_mA + this.m_iC * JwC * JwC + this.m_iA * JwA * JwA;

            var pC = b2Math.SubtractVV(this.m_localAnchorC, this.m_lcC);
            var pA = b2Math.MulTRV(qC, b2Math.AddVV(rA, b2Math.SubtractVV(cA, cC)));
            coordinateA = b2Math.DotVV(b2Math.SubtractVV(pA, pC), this.m_localAxisC);
        }

        if (this.m_typeB == b2Joint.e_revoluteJoint) {
            JvBD.SetZero();
            JwB = this.m_ratio;
            JwD = this.m_ratio;
            mass += this.m_ratio * this.m_ratio * (this.m_iB + this.m_iD);

            coordinateB = aB - aD - this.m_referenceAngleB;
        } else {
            var u = b2Math.MulRV(qD, this.m_localAxisD);
            var rD = b2Math.MulRV(qD, b2Math.SubtractVV(this.m_localAnchorD, this.m_lcD));
            var rB = b2Math.MulRV(qB, b2Math.SubtractVV(this.m_localAnchorB, this.m_lcB));
            JvBD = b2Math.MulFV(this.m_ratio, u);
            JwD = this.m_ratio * b2Math.CrossVV(rD, u);
            JwB = this.m_ratio * b2Math.CrossVV(rB, u);
            mass += this.m_ratio * this.m_ratio * (this.m_mD + this.m_mB) + this.m_iD * JwD * JwD + this.m_iB * JwB * JwB;

            var pD = b2Math.SubtractVV(this.m_localAnchorD, this.m_lcD);
            var pB = b2Math.MulTRV(qD, b2Math.AddVV(rB, b2Math.SubtractVV(cB, cD)));
            coordinateB = b2Math.DotVV(b2Math.SubtractVV(pB, pD), this.m_localAxisD);
        }

        var C = (coordinateA + this.m_ratio * coordinateB) - this.m_constant;

        var impulse = 0.0;
        if (mass > 0.0) {
            impulse = -C / mass;
        }

        cA.Add(b2Math.MulFV(this.m_mA * impulse, JvAC));
        aA += this.m_iA * impulse * JwA;
        cB.Add(b2Math.MulFV(this.m_mB * impulse, JvBD));
        aB += this.m_iB * impulse * JwB;
        cC.Subtract(b2Math.MulFV(this.m_mC * impulse, JvAC));
        aC -= this.m_iC * impulse * JwC;
        cD.Subtract(b2Math.MulFV(this.m_mD * impulse, JvBD));
        aD -= this.m_iD * impulse * JwD;

//        data.positions[this.m_indexA].c = cA.Copy();
        data.positions[this.m_indexA].a = aA;
//        data.positions[this.m_indexB].c = cB.Copy();
        data.positions[this.m_indexB].a = aB;
//        data.positions[this.m_indexC].c = cC.Copy();
        data.positions[this.m_indexC].a = aC;
//        data.positions[this.m_indexD].c = cD.Copy();
        data.positions[this.m_indexD].a = aD;

        // TODO_ERIN not implemented
        return linearError < b2Settings.b2_linearSlop;
    }
    Box2D.inherit(b2GearJointDef, Box2D.Dynamics.Joints.b2JointDef);
    b2GearJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
    b2GearJointDef.b2GearJointDef = function() {
        Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
    };
    b2GearJointDef.prototype.b2GearJointDef = function() {
        this.__super.b2JointDef.call(this);
        this.type = b2Joint.e_gearJoint;
        this.joint1 = null;
        this.joint2 = null;
        this.ratio = 1.0;
    }
    b2Jacobian.b2Jacobian = function() {
        this.linear = new b2Vec2();
        this.angularA = 0.0;
        this.angularB = 0.0;
    };
    b2Joint.b2Joint = function() {
        this.m_edgeA = new b2JointEdge();
        this.m_edgeB = new b2JointEdge();
    };
    b2Joint.prototype.GetType = function() {
        return this.m_type;
    }
    b2Joint.prototype.GetAnchorA = function() {
        return null;
    }
    b2Joint.prototype.GetAnchorB = function() {
        return null;
    }
    b2Joint.prototype.GetReactionForce = function(inv_dt) {
        return null;
    }
    b2Joint.prototype.GetReactionTorque = function(inv_dt) {
        return 0.0;
    }
    b2Joint.prototype.GetBodyA = function() {
        return this.m_bodyA;
    }
    b2Joint.prototype.GetBodyB = function() {
        return this.m_bodyB;
    }
    b2Joint.prototype.GetNext = function() {
        return this.m_next;
    }
    b2Joint.prototype.GetUserData = function() {
        return this.m_userData;
    }
    b2Joint.prototype.SetUserData = function(data) {
        this.m_userData = data;
    }
    b2Joint.prototype.IsActive = function() {
        return this.m_bodyA.IsActive() && this.m_bodyB.IsActive();
    }
    b2Joint.prototype.GetCollideConnected = function() {
        return this.m_collideConnected;
    }
    b2Joint.Create = function(def, allocator) {
        var joint = null;
        switch (def.type) {
        case b2Joint.e_distanceJoint:
            {
                joint = new b2DistanceJoint((def instanceof b2DistanceJointDef ? def : null));
            }
            break;
        case b2Joint.e_mouseJoint:
            {
                joint = new b2MouseJoint((def instanceof b2MouseJointDef ? def : null));
            }
            break;
        case b2Joint.e_prismaticJoint:
            {
                joint = new b2PrismaticJoint((def instanceof b2PrismaticJointDef ? def : null));
            }
            break;
        case b2Joint.e_revoluteJoint:
            {
                joint = new b2RevoluteJoint((def instanceof b2RevoluteJointDef ? def : null));
            }
            break;
        case b2Joint.e_pulleyJoint:
            {
                joint = new b2PulleyJoint((def instanceof b2PulleyJointDef ? def : null));
            }
            break;
        case b2Joint.e_gearJoint:
            {
                joint = new b2GearJoint((def instanceof b2GearJointDef ? def : null));
            }
            break;
        case b2Joint.e_weldJoint:
            {
                joint = new b2WeldJoint((def instanceof b2WeldJointDef ? def : null));
            }
            break;
        case b2Joint.e_wheelJoint:
            {
                joint = new b2WheelJoint((def instanceof b2WheelJointDef ? def : null));
            }
            break;
        case b2Joint.e_frictionJoint:
            {
                joint = new b2FrictionJoint((def instanceof b2FrictionJointDef ? def : null));
            }
            break;
        case b2Joint.e_ropeJoint:
            {
                joint = new b2RopeJoint((def instanceof b2RopeJointDef ? def : null));
            }
            break;
        default:
            b2Settings.b2Assert(0);
            break;
        }
        return joint;
    }
    b2Joint.Destroy = function(joint, allocator) {}
    b2Joint.prototype.b2Joint = function(def) {
        b2Settings.b2Assert(def.bodyA != def.bodyB);
        this.m_type = def.type;
        this.m_prev = null;
        this.m_next = null;
        this.m_bodyA = def.bodyA;
        this.m_bodyB = def.bodyB;
        this.m_index = 0;
        this.m_collideConnected = def.collideConnected;
        this.m_islandFlag = false;
        this.m_userData = def.userData;

        this.m_edgeA.joint = null;
        this.m_edgeA.other = null;
        this.m_edgeA.prev = null;
        this.m_edgeA.next = null;
        this.m_edgeB.joint = null;
        this.m_edgeB.other = null;
        this.m_edgeB.prev = null;
        this.m_edgeB.next = null;
    }
    b2Joint.prototype.InitVelocityConstraints = function(data) {}
    b2Joint.prototype.SolveVelocityConstraints = function(data) {}
    b2Joint.prototype.SolvePositionConstraints = function(data) {
        return false;
    }
    Box2D.postDefs.push(function() {
        Box2D.Dynamics.Joints.b2Joint.e_unknownJoint = 0;
        Box2D.Dynamics.Joints.b2Joint.e_revoluteJoint = 1;
        Box2D.Dynamics.Joints.b2Joint.e_prismaticJoint = 2;
        Box2D.Dynamics.Joints.b2Joint.e_distanceJoint = 3;
        Box2D.Dynamics.Joints.b2Joint.e_pulleyJoint = 4;
        Box2D.Dynamics.Joints.b2Joint.e_mouseJoint = 5;
        Box2D.Dynamics.Joints.b2Joint.e_gearJoint = 6;
        Box2D.Dynamics.Joints.b2Joint.e_wheelJoint = 7;
        Box2D.Dynamics.Joints.b2Joint.e_weldJoint = 8;
        Box2D.Dynamics.Joints.b2Joint.e_frictionJoint = 9;
        Box2D.Dynamics.Joints.b2Joint.e_ropeJoint = 10;

        Box2D.Dynamics.Joints.b2Joint.e_inactiveLimit = 0;
        Box2D.Dynamics.Joints.b2Joint.e_atLowerLimit = 1;
        Box2D.Dynamics.Joints.b2Joint.e_atUpperLimit = 2;
        Box2D.Dynamics.Joints.b2Joint.e_equalLimits = 3;
    });
    b2JointDef.b2JointDef = function() {};
    b2JointDef.prototype.b2JointDef = function() {
        this.type = b2Joint.e_unknownJoint;
        this.userData = null;
        this.bodyA = null;
        this.bodyB = null;
        this.collideConnected = false;
    }
    b2JointEdge.b2JointEdge = function() {
        this.other = null;
        this.next = null;
        this.prev = null;
        this.joint = null;
    };
    Box2D.inherit(b2MouseJoint, Box2D.Dynamics.Joints.b2Joint);
    b2MouseJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
    b2MouseJoint.b2MouseJoint = function() {
        Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
        this.m_localAnchorB = new b2Vec2();
        this.m_targetA = new b2Vec2();
        this.m_impulse = new b2Vec2();
        this.m_rB = new b2Vec2();
        this.m_localCenterB = new b2Vec2();
        this.m_mass = new b2Mat22();
        this.m_C = new b2Vec2();
    };
    b2MouseJoint.prototype.GetAnchorA = function() {
        return this.m_targetA;
    }
    b2MouseJoint.prototype.GetAnchorB = function() {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
    }
    b2MouseJoint.prototype.GetReactionForce = function(inv_dt) {
        return new b2Vec2(inv_dt * this.m_impulse.x, inv_dt * this.m_impulse.y);
    }
    b2MouseJoint.prototype.GetReactionTorque = function(inv_dt) {
        return 0.0;
    }
    b2MouseJoint.prototype.GetTarget = function() {
        return this.m_targetA;
    }
    b2MouseJoint.prototype.SetTarget = function(target) {
        if (this.m_bodyB.IsAwake() == false) {
            this.m_bodyB.SetAwake(true);
        }
        this.m_targetA = target;
    }
    b2MouseJoint.prototype.GetMaxForce = function() {
        return this.m_maxForce;
    }
    b2MouseJoint.prototype.SetMaxForce = function(maxForce) {
        this.m_maxForce = maxForce;
    }
    b2MouseJoint.prototype.GetFrequency = function() {
        return this.m_frequencyHz;
    }
    b2MouseJoint.prototype.SetFrequency = function(hz) {
        this.m_frequencyHz = hz;
    }
    b2MouseJoint.prototype.GetDampingRatio = function() {
        return this.m_dampingRatio;
    }
    b2MouseJoint.prototype.SetDampingRatio = function(ratio) {
        this.m_dampingRatio = ratio;
    }
    b2MouseJoint.prototype.b2MouseJoint = function(def) {
        this.__super.b2Joint.call(this, def);

        this.m_targetA.SetV(def.target);
        this.m_localAnchorB = b2Math.MulTXV(this.m_bodyB.GetTransform(), this.m_targetA);

        this.m_maxForce = def.maxForce;
        this.m_impulse.SetZero();
        this.m_frequencyHz = def.frequencyHz;
        this.m_dampingRatio = def.dampingRatio;
        this.m_beta = 0.0;
        this.m_gamma = 0.0;
    }
    b2MouseJoint.prototype.InitVelocityConstraints = function(data) {
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_localCenterB.SetV(this.m_bodyB.m_sweep.localCenter);
        this.m_invMassB = this.m_bodyB.m_invMass;
        this.m_invIB = this.m_bodyB.m_invI;

        var cB = data.positions[this.m_indexB].c;
        var aB = data.positions[this.m_indexB].a;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;

        var qB = new b2Rot(aB);

        var mass = this.m_bodyB.GetMass();

        // Frequency
        var omega = 2.0 * b2Settings.b2_pi * this.m_frequencyHz;

        // Damping coefficient
        var d = 2.0 * mass * this.m_dampingRatio * omega;

        // Spring stiffness
        var k = mass * (omega * omega);

        // magic formulas
        // gamma has units of inverse mass.
        // beta has units of inverse time.
        var h = data.step.dt;
        b2Settings.b2Assert(d + h * k > b2Settings.b2_epsilon);
        this.m_gamma = h * (d + h * k);
        if (this.m_gamma != 0.0) {
            this.m_gamma = 1.0 / this.m_gamma;
        }
        this.m_beta = h * k * this.m_gamma;

        // Compute the effective mass matrix.
        this.m_rB = b2Math.MulRV(qB, b2Math.SubtractVV(this.m_localAnchorB, this.m_localCenterB));

        // K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
        //      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
        //        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
        var K = new b2Mat22();
        K.ex.x = this.m_invMassB + this.m_invIB * this.m_rB.y * this.m_rB.y + this.m_gamma;
        K.ex.y = -this.m_invIB * this.m_rB.x * this.m_rB.y;
        K.ey.x = K.ex.y;
        K.ey.y = this.m_invMassB + this.m_invIB * this.m_rB.x * this.m_rB.x + this.m_gamma;

        this.m_mass = K.GetInverse();

        this.m_C = b2Math.AddVV(cB, b2Math.SubtractVV(this.m_rB, this.m_targetA));
        this.m_C.Multiply(this.m_beta);

        // Cheat with some damping
        wB *= 0.98;

        if (data.step.warmStarting) {
            this.m_impulse.Multiply(data.step.dtRatio);
            vB.Add(b2Math.MulFV(this.m_invMassB, this.m_impulse));
            wB += this.m_invIB * b2Math.CrossVV(this.m_rB, this.m_impulse);
        } else {
            this.m_impulse.SetZero();
        }

        data.velocities[this.m_indexB].v.SetV(vB);
        data.velocities[this.m_indexB].w = wB;
    }
    b2MouseJoint.prototype.SolveVelocityConstraints = function(data) {
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;

        var Cdot = b2Math.AddVV(vB, b2Math.CrossFV(wB, this.m_rB));
        var tmp = b2Math.AddVV(Cdot, this.m_C);
        tmp.Add(b2Math.MulFV(this.m_gamma, this.m_impulse));
        var impulse = b2Math.MulMV(this.m_mass, tmp.GetNegative());

        var oldImpulse = this.m_impulse.Copy();
        this.m_impulse.Add(impulse);
        var maxImpulse = data.step.dt * this.m_maxForce;
        if (this.m_impulse.LengthSquared() > maxImpulse * maxImpulse) {
            this.m_impulse *= maxImpulse / this.m_impulse.Length();
        }
        impulse = this.m_impulse - oldImpulse;

        vB.Add(b2Math.MulFV(this.m_invMassB, impulse));
        wB += this.m_invIB * b2Math.CrossVV(this.m_rB, impulse);

//        data.velocities[this.m_indexB].v = vB.Copy();
        data.velocities[this.m_indexB].w = wB;
    }
    b2MouseJoint.prototype.SolvePositionConstraints = function(data) {
        return true;
    }
    Box2D.inherit(b2MouseJointDef, Box2D.Dynamics.Joints.b2JointDef);
    b2MouseJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
    b2MouseJointDef.b2MouseJointDef = function() {
        Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
        this.target = new b2Vec2();
    };
    b2MouseJointDef.prototype.b2MouseJointDef = function() {
        this.__super.b2JointDef.call(this);
        this.type = b2Joint.e_mouseJoint;
        this.maxForce = 0.0;
        this.frequencyHz = 5.0;
        this.dampingRatio = 0.7;
    }
    Box2D.inherit(b2PrismaticJoint, Box2D.Dynamics.Joints.b2Joint);
    b2PrismaticJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
    b2PrismaticJoint.b2PrismaticJoint = function() {
        Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
        this.m_localAnchorA = new b2Vec2();
        this.m_localAnchorB = new b2Vec2();
        this.m_localXAxisA = new b2Vec2();
        this.m_localYAxisA = new b2Vec2();
        this.m_impulse = new b2Vec3();
        this.m_localCenterA = new b2Vec2();
        this.m_localCenterB = new b2Vec2();
        this.m_axis = new b2Vec2();
        this.m_perp = new b2Vec2();
        this.m_K = new b2Mat33();
    };
    b2PrismaticJoint.prototype.GetLocalAnchorA = function() {
        return this.m_localAnchorA;
    }
    b2PrismaticJoint.prototype.GetLocalAnchorB = function() {
        return this.m_localAnchorB;
    }
    b2PrismaticJoint.prototype.GetLocalAxisA = function() {
        return this.m_localXAxisA;
    }
    b2PrismaticJoint.prototype.GetAnchorA = function() {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
    }
    b2PrismaticJoint.prototype.GetAnchorB = function() {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
    }
    b2PrismaticJoint.prototype.GetReactionForce = function(inv_dt) {
        return new b2Vec2(inv_dt * (this.m_impulse.x * this.m_perp.x + (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.x), inv_dt * (this.m_impulse.x * this.m_perp.y + (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.y));
    }
    b2PrismaticJoint.prototype.GetReactionTorque = function(inv_dt) {
        return inv_dt * this.m_impulse.y;
    }
    b2PrismaticJoint.prototype.GetJointTranslation = function() {
        var bA = this.m_bodyA;
        var bB = this.m_bodyB;
        var p1 = bA.GetWorldPoint(this.m_localAnchorA);
        var p2 = bB.GetWorldPoint(this.m_localAnchorB);
        var dX = p2.x - p1.x;
        var dY = p2.y - p1.y;
        var axis = bA.GetWorldVector(this.m_localXAxisA);
        var translation = axis.x * dX + axis.y * dY;
        return translation;
    }
    b2PrismaticJoint.prototype.GetJointSpeed = function() {
        var bA = this.m_bodyA;
        var bB = this.m_bodyB;
        var tRot;
        tRot = bA.m_xf.q;
        var r1X = this.m_localAnchorA.x - bA.m_sweep.localCenter.x;
        var r1Y = this.m_localAnchorA.y - bA.m_sweep.localCenter.y;
        var tX = (tRot.c * r1X - tRot.s * r1Y);
        r1Y = (tRot.s * r1X + tRot.c * r1Y);
        r1X = tX;
        tRot = bB.m_xf.q;
        var r2X = this.m_localAnchorB.x - bB.m_sweep.localCenter.x;
        var r2Y = this.m_localAnchorB.y - bB.m_sweep.localCenter.y;
        tX = (tRot.c * r2X - tRot.s * r2Y);
        r2Y = (tRot.s * r2X + tRot.c * r2Y);
        r2X = tX;
        var p1X = bA.m_sweep.c.x + r1X;
        var p1Y = bA.m_sweep.c.y + r1Y;
        var p2X = bB.m_sweep.c.x + r2X;
        var p2Y = bB.m_sweep.c.y + r2Y;
        var dX = p2X - p1X;
        var dY = p2Y - p1Y;
        var axis = bA.GetWorldVector(this.m_localXAxisA);
        var v1 = bA.m_linearVelocity;
        var v2 = bB.m_linearVelocity;
        var w1 = bA.m_angularVelocity;
        var w2 = bB.m_angularVelocity;
        var speed = (dX * ((-w1 * axis.y)) + dY * (w1 * axis.x)) + (axis.x * (((v2.x + ((-w2 * r2Y))) - v1.x) - ((-w1 * r1Y))) + axis.y * (((v2.y + (w2 * r2X)) - v1.y) - (w1 * r1X)));
        return speed;
    }
    b2PrismaticJoint.prototype.IsLimitEnabled = function() {
        return this.m_enableLimit;
    }
    b2PrismaticJoint.prototype.EnableLimit = function(flag) {
        if (flag != this.m_enableLimit) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_enableLimit = flag;
            this.m_impulse.z = 0.0;
        }
    }
    b2PrismaticJoint.prototype.GetLowerLimit = function() {
        return this.m_lowerTranslation;
    }
    b2PrismaticJoint.prototype.GetUpperLimit = function() {
        return this.m_upperTranslation;
    }
    b2PrismaticJoint.prototype.SetLimits = function(lower, upper) {
        if (lower != this.m_lowerTranslation || upper != this.m_upperTranslation) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_lowerTranslation = lower;
            this.m_upperTranslation = upper;
            this.m_impulse.z = 0.0;
        }
    }
    b2PrismaticJoint.prototype.IsMotorEnabled = function() {
        return this.m_enableMotor;
    }
    b2PrismaticJoint.prototype.EnableMotor = function(flag) {
        this.m_bodyA.SetAwake(true);
        this.m_bodyB.SetAwake(true);
        this.m_enableMotor = flag;
    }
    b2PrismaticJoint.prototype.SetMotorSpeed = function(speed) {
        this.m_bodyA.SetAwake(true);
        this.m_bodyB.SetAwake(true);
        this.m_motorSpeed = speed;
    }
    b2PrismaticJoint.prototype.GetMotorSpeed = function() {
        return this.m_motorSpeed;
    }
    b2PrismaticJoint.prototype.SetMaxMotorForce = function(force) {
        this.m_bodyA.SetAwake(true);
        this.m_bodyB.SetAwake(true);
        this.m_maxMotorForce = force;
    }
    b2PrismaticJoint.prototype.GetMotorForce = function(inv_dt) {
        return inv_dt * this.m_motorImpulse;
    }
    b2PrismaticJoint.prototype.GetReferenceAngle = function() {
        return this.m_referenceAngle;
    }
    b2PrismaticJoint.prototype.b2PrismaticJoint = function(def) {
        this.__super.b2Joint.call(this, def);
        this.m_localAnchorA.SetV(def.localAnchorA);
        this.m_localAnchorB.SetV(def.localAnchorB);
        this.m_localXAxisA.SetV(def.localAxisA);
        this.m_localXAxisA.Normalize();
        this.m_localYAxisA.x = (-this.m_localXAxisA.y);
        this.m_localYAxisA.y = this.m_localXAxisA.x;
        this.m_referenceAngle = def.referenceAngle;
        this.m_impulse.SetZero();
        this.m_motorMass = 0.0;
        this.m_motorImpulse = 0.0;
        this.m_lowerTranslation = def.lowerTranslation;
        this.m_upperTranslation = def.upperTranslation;
        this.m_maxMotorForce = def.maxMotorForce;
        this.m_motorSpeed = def.motorSpeed;
        this.m_enableLimit = def.enableLimit;
        this.m_enableMotor = def.enableMotor;
        this.m_limitState = b2Joint.e_inactiveLimit;
        this.m_axis.SetZero();
        this.m_perp.SetZero();
    }
    b2PrismaticJoint.prototype.InitVelocityConstraints = function(data) {
        this.m_indexA = this.m_bodyA.m_islandIndex;
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_localCenterA.SetV(this.m_bodyA.m_sweep.localCenter);
        this.m_localCenterB.SetV(this.m_bodyB.m_sweep.localCenter);
        this.m_invMassA = this.m_bodyA.m_invMass;
        this.m_invMassB = this.m_bodyB.m_invMass;
        this.m_invIA = this.m_bodyA.m_invI;
        this.m_invIB = this.m_bodyB.m_invI;

        var cA = data.positions[this.m_indexA].c;
        var aA = data.positions[this.m_indexA].a;
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;

        var cB = data.positions[this.m_indexB].c;
        var aB = data.positions[this.m_indexB].a;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;

        var qA = new b2Rot(aA), qB = new b2Rot(aB);

        // Compute the effective masses.
        var rA = b2Math.MulRV(qA, b2Math.SubtractVV(this.m_localAnchorA, this.m_localCenterA));
        var rB = b2Math.MulRV(qB, b2Math.SubtractVV(this.m_localAnchorB, this.m_localCenterB));
        var d = b2Math.AddVV(b2Math.SubtractVV(cB, cA), b2Math.SubtractVV(rB, rA));

        var mA = this.m_invMassA, mB = this.m_invMassB;
        var iA = this.m_invIA, iB = this.m_invIB;

        // Compute motor Jacobian and effective mass.
        {
            this.m_axis = b2Math.MulRV(qA, this.m_localXAxisA);
            this.m_a1 = b2Math.CrossVV(b2Math.AddVV(d, rA), this.m_axis);
            this.m_a2 = b2Math.CrossVV(rB, this.m_axis);

            this.m_motorMass = mA + mB + iA * this.m_a1 * this.m_a1 + iB * this.m_a2 * this.m_a2;
            if (this.m_motorMass > 0.0) {
                this.m_motorMass = 1.0 / this.m_motorMass;
            }
        }

        // Prismatic constraint.
        {
            this.m_perp = b2Math.MulRV(qA, this.m_localYAxisA);

            this.m_s1 = b2Math.CrossVV(b2Math.AddVV(d, rA), this.m_perp);
            this.m_s2 = b2Math.CrossVV(rB, this.m_perp);

            var k11 = mA + mB + iA * this.m_s1 * this.m_s1 + iB * this.m_s2 * this.m_s2;
            var k12 = iA * this.m_s1 + iB * this.m_s2;
            var k13 = iA * this.m_s1 * this.m_a1 + iB * this.m_s2 * this.m_a2;
            var k22 = iA + iB;
            if (k22 == 0.0) {
                // For bodies with fixed rotation.
                k22 = 1.0;
            }
            var k23 = iA * this.m_a1 + iB * this.m_a2;
            var k33 = mA + mB + iA * this.m_a1 * this.m_a1 + iB * this.m_a2 * this.m_a2;

            this.m_K.ex.Set(k11, k12, k13);
            this.m_K.ey.Set(k12, k22, k23);
            this.m_K.ez.Set(k13, k23, k33);
        }

        // Compute motor and limit terms.
        if (this.m_enableLimit) {
            var jointTranslation = b2Math.DotVV(this.m_axis, d);
            if (b2Math.Abs(this.m_upperTranslation - this.m_lowerTranslation) < 2.0 * b2Settings.b2_linearSlop) {
                this.m_limitState = b2Joint.e_equalLimits;
            } else if (jointTranslation <= this.m_lowerTranslation) {
                if (this.m_limitState != b2Joint.e_atLowerLimit) {
                    this.m_limitState = b2Joint.e_atLowerLimit;
                    this.m_impulse.z = 0.0;
                }
            } else if (jointTranslation >= this.m_upperTranslation) {
                if (this.m_limitState != b2Joint.e_atUpperLimit) {
                    this.m_limitState = b2Joint.e_atUpperLimit;
                    this.m_impulse.z = 0.0;
                }
            } else {
                this.m_limitState = b2Joint.e_inactiveLimit;
                this.m_impulse.z = 0.0;
            }
        } else {
            this.m_limitState = b2Joint.e_inactiveLimit;
            this.m_impulse.z = 0.0;
        }

        if (this.m_enableMotor == false) {
            this.m_motorImpulse = 0.0;
        }

        if (data.step.warmStarting) {
            // Account for variable time step.
            this.m_impulse.Multiply(data.step.dtRatio);
            this.m_motorImpulse *= data.step.dtRatio;

            var P = b2Math.AddVV(b2Math.MulFV(this.m_impulse.x, this.m_perp), b2Math.MulFV(this.m_motorImpulse + this.m_impulse.z, this.m_axis));
            var LA = this.m_impulse.x * this.m_s1 + this.m_impulse.y + (this.m_motorImpulse + this.m_impulse.z) * this.m_a1;
            var LB = this.m_impulse.x * this.m_s2 + this.m_impulse.y + (this.m_motorImpulse + this.m_impulse.z) * this.m_a2;

            vA.Subtract(b2Math.MulFV(mA, P));
            wA -= iA * LA;

            vB.Add(b2Math.MulFV(mB, P));
            wB += iB * LB;
        } else {
            this.m_impulse.SetZero();
            this.m_motorImpulse = 0.0;
        }

//        data.velocities[this.m_indexA].v.SetV(vA);
        data.velocities[this.m_indexA].w = wA;
//        data.velocities[this.m_indexB].v.SetV(vB);
        data.velocities[this.m_indexB].w = wB;
    }
    b2PrismaticJoint.prototype.SolveVelocityConstraints = function(data) {
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;

        var mA = this.m_invMassA, mB = this.m_invMassB;
        var iA = this.m_invIA, iB = this.m_invIB;

        // Solve linear motor constraint.
        if (this.m_enableMotor && this.m_limitState != b2Joint.e_equalLimits) {
            var Cdot = b2Math.DotVV(this.m_axis, b2Math.SubtractVV(vB, vA)) + this.m_a2 * wB - this.m_a1 * wA;
            var impulse = this.m_motorMass * (this.m_motorSpeed - Cdot);
            var oldImpulse = this.m_motorImpulse;
            var maxImpulse = data.step.dt * this.m_maxMotorForce;
            this.m_motorImpulse = b2Math.Clamp(this.m_motorImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = this.m_motorImpulse - oldImpulse;

            var P = b2Math.MulFV(impulse, this.m_axis);
            var LA = impulse * this.m_a1;
            var LB = impulse * this.m_a2;

            vA.Subtract(b2Math.MulFV(mA, P));
            wA -= iA * LA;

            vB.Add(b2Math.MulFV(mB, P));
            wB += iB * LB;
        }

        var Cdot1 = new b2Vec2();
        Cdot1.x = b2Math.DotVV(this.m_perp, b2Math.SubtractVV(vB, vA)) + this.m_s2 * wB - this.m_s1 * wA;
        Cdot1.y = wB - wA;

        if (this.m_enableLimit && this.m_limitState != b2Joint.e_inactiveLimit) {
            // Solve prismatic and limit constraint in block form.
            var Cdot2;
            Cdot2 = b2Math.DotVV(this.m_axis, b2Math.SubtractVV(vB, vA)) + this.m_a2 * wB - this.m_a1 * wA;
            var Cdot = new b2Vec3(Cdot1.x, Cdot1.y, Cdot2);

            var f1 = this.m_impulse.Copy();
            var df = this.m_K.Solve33(Cdot.GetNegative());
            this.m_impulse.Add(df);

            if (this.m_limitState == b2Joint.e_atLowerLimit) {
                this.m_impulse.z = b2Math.Max(this.m_impulse.z, 0.0);
            } else if (this.m_limitState == b2Joint.e_atUpperLimit) {
                this.m_impulse.z = b2Math.Min(this.m_impulse.z, 0.0);
            }

            // f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
            var b = new b2Vec2(this.m_K.ez.x, this.m_K.ez.y);
            b.Multiply(- (this.m_impulse.z - f1.z));
            b.Subtract(Cdot1);
            var f2r = new b2Vec2(f1.x, f1.y);
            f2r.Add(this.m_K.Solve22(b));
            this.m_impulse.x = f2r.x;
            this.m_impulse.y = f2r.y;

            df = b2Math.SubtractVV3(this.m_impulse, f1);

            var P = b2Math.AddVV(b2Math.MulFV(df.x, this.m_perp), b2Math.MulFV(df.z, this.m_axis));
            var LA = df.x * this.m_s1 + df.y + df.z * this.m_a1;
            var LB = df.x * this.m_s2 + df.y + df.z * this.m_a2;

            vA.Subtract(b2Math.MulFV(mA, P));
            wA -= iA * LA;

            vB.Add(b2Math.MulFV(mB, P));
            wB += iB * LB;
        } else {
            // Limit is inactive, just solve the prismatic constraint in block form.
            var df = this.m_K.Solve22(Cdot1.GetNegative());
            this.m_impulse.x += df.x;
            this.m_impulse.y += df.y;

            var P = b2Math.MulFV(df.x, this.m_perp);
            var LA = df.x * this.m_s1 + df.y;
            var LB = df.x * this.m_s2 + df.y;

            vA.Subtract(b2Math.MulFV(mA, P));
            wA -= iA * LA;

            vB.Add(b2Math.MulFV(mB, P));
            wB += iB * LB;

            var Cdot10 = Cdot1.Copy();

            Cdot1.x = b2Math.DotVV(this.m_perp, b2Math.SubtractVV(vB, vA)) + this.m_s2 * wB - this.m_s1 * wA;
            Cdot1.y = wB - wA;

            if (b2Math.Abs(Cdot1.x) > 0.01 || b2Math.Abs(Cdot1.y) > 0.01) {
                var test = b2Math.MulMV(this.m_K, df);
                Cdot1.x += 0.0;
            }
        }

//        data.velocities[this.m_indexA].v.SetV(vA);
        data.velocities[this.m_indexA].w = wA;
//        data.velocities[this.m_indexB].v.SetV(vB);
        data.velocities[this.m_indexB].w = wB;
    }
    b2PrismaticJoint.prototype.SolvePositionConstraints = function(data) {
        var cA = data.positions[this.m_indexA].c;
        var aA = data.positions[this.m_indexA].a;
        var cB = data.positions[this.m_indexB].c;
        var aB = data.positions[this.m_indexB].a;

        var qA = new b2Rot(aA), qB = new b2Rot(aB);

        var mA = this.m_invMassA, mB = this.m_invMassB;
        var iA = this.m_invIA, iB = this.m_invIB;

        // Compute fresh Jacobians
        var rA = b2Math.MulRV(qA, b2Math.SubtractVV(this.m_localAnchorA, this.m_localCenterA));
        var rB = b2Math.MulRV(qB, b2Math.SubtractVV(this.m_localAnchorB, this.m_localCenterB));
        var d = b2Math.SubtractVV(b2Math.AddVV(cB, rB), b2Math.AddVV(cA, rA));

        var axis = b2Math.MulRV(qA, this.m_localXAxisA);
        var a1 = b2Math.CrossVV(b2Math.AddVV(d, rA), axis);
        var a2 = b2Math.CrossVV(rB, axis);
        var perp = b2Math.MulRV(qA, this.m_localYAxisA);

        var s1 = b2Math.CrossVV(b2Math.AddVV(d, rA), perp);
        var s2 = b2Math.CrossVV(rB, perp);

        var impulse = new b2Vec3();
        var C1 = new b2Vec2();
        C1.x = b2Math.DotVV(perp, d);
        C1.y = aB - aA - this.m_referenceAngle;

        var linearError = b2Math.Abs(C1.x);
        var angularError = b2Math.Abs(C1.y);

        var active = false;
        var C2 = 0.0;
        if (this.m_enableLimit) {
            var translation = b2Math.DotVV(axis, d);
            if (b2Math.Abs(this.m_upperTranslation - this.m_lowerTranslation) < 2.0 * b2Settings.b2_linearSlop) {
                // Prevent large angular corrections
                C2 = b2Math.Clamp(translation, -b2Settings.b2_maxLinearCorrection, b2Settings.b2_maxLinearCorrection);
                linearError = b2Math.Max(linearError, b2Math.Abs(translation));
                active = true;
            } else if (translation <= this.m_lowerTranslation) {
                // Prevent large linear corrections and allow some slop.
                C2 = b2Math.Clamp(translation - this.m_lowerTranslation + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0.0);
                linearError = b2Math.Max(linearError, this.m_lowerTranslation - translation);
                active = true;
            } else if (translation >= this.m_upperTranslation) {
                // Prevent large linear corrections and allow some slop.
                C2 = b2Math.Clamp(translation - this.m_upperTranslation - b2Settings.b2_linearSlop, 0.0, b2Settings.b2_maxLinearCorrection);
                linearError = b2Math.Max(linearError, translation - this.m_upperTranslation);
                active = true;
            }
        }

        if (active) {
            var k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
            var k12 = iA * s1 + iB * s2;
            var k13 = iA * s1 * a1 + iB * s2 * a2;
            var k22 = iA + iB;
            if (k22 == 0.0) {
                // For fixed rotation
                k22 = 1.0;
            }
            var k23 = iA * a1 + iB * a2;
            var k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;

            var K = new b2Mat33();
            K.ex.Set(k11, k12, k13);
            K.ey.Set(k12, k22, k23);
            K.ez.Set(k13, k23, k33);

            var C = new b2Vec3();
            C.x = C1.x;
            C.y = C1.y;
            C.z = C2;

            impulse = K.Solve33(C.GetNegative());
        } else {
            var k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
            var k12 = iA * s1 + iB * s2;
            var k22 = iA + iB;
            if (k22 == 0.0) {
                k22 = 1.0;
            }

            var K = new b2Mat22();
            K.ex.Set(k11, k12);
            K.ey.Set(k12, k22);

            var impulse1 = K.Solve(C1.GetNegative());
            impulse.x = impulse1.x;
            impulse.y = impulse1.y;
            impulse.z = 0.0;
        }

        var P = b2Math.AddVV(b2Math.MulFV(impulse.x, perp), b2Math.MulFV(impulse.z, axis));
        var LA = impulse.x * s1 + impulse.y + impulse.z * a1;
        var LB = impulse.x * s2 + impulse.y + impulse.z * a2;

        cA.Subtract(b2Math.MulFV(mA, P));
        aA -= iA * LA;
        cB.Add(b2Math.MulFV(mB, P));
        aB += iB * LB;

//        data.positions[this.m_indexA].c.SetV(cA);
        data.positions[this.m_indexA].a = aA;
//        data.positions[this.m_indexB].c.SetV(cB);
        data.positions[this.m_indexB].a = aB;

        return linearError <= b2Settings.b2_linearSlop && angularError <= b2Settings.b2_angularSlop;
    }
    Box2D.inherit(b2PrismaticJointDef, Box2D.Dynamics.Joints.b2JointDef);
    b2PrismaticJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
    b2PrismaticJointDef.b2PrismaticJointDef = function() {
        Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
        this.localAnchorA = new b2Vec2();
        this.localAnchorB = new b2Vec2();
        this.localAxisA = new b2Vec2();
    };
    b2PrismaticJointDef.prototype.b2PrismaticJointDef = function() {
        this.__super.b2JointDef.call(this);
        this.type = b2Joint.e_prismaticJoint;
        this.localAxisA.Set(1.0, 0.0);
        this.referenceAngle = 0.0;
        this.enableLimit = false;
        this.lowerTranslation = 0.0;
        this.upperTranslation = 0.0;
        this.enableMotor = false;
        this.maxMotorForce = 0.0;
        this.motorSpeed = 0.0;
    }
    b2PrismaticJointDef.prototype.Initialize = function(bA, bB, anchor, axis) {
        this.bodyA = bA;
        this.bodyB = bB;
        this.localAnchorA = this.bodyA.GetLocalPoint(anchor);
        this.localAnchorB = this.bodyB.GetLocalPoint(anchor);
        this.localAxisA = this.bodyA.GetLocalVector(axis);
        this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
    }
    Box2D.inherit(b2PulleyJoint, Box2D.Dynamics.Joints.b2Joint);
    b2PulleyJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
    b2PulleyJoint.b2PulleyJoint = function() {
        Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
        this.m_groundAnchorA = new b2Vec2();
        this.m_groundAnchorB = new b2Vec2();
        this.m_localAnchorA = new b2Vec2();
        this.m_localAnchorB = new b2Vec2();
        this.m_uA = new b2Vec2();
        this.m_uB = new b2Vec2();
        this.m_rA = new b2Vec2();
        this.m_rB = new b2Vec2();
        this.m_localCenterA = new b2Vec2();
        this.m_localCenterB = new b2Vec2();
    };
    b2PulleyJoint.prototype.GetAnchorA = function() {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
    }
    b2PulleyJoint.prototype.GetAnchorB = function() {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
    }
    b2PulleyJoint.prototype.GetReactionForce = function(inv_dt) {
        if (inv_dt === undefined) inv_dt = 0;
        return new b2Vec2(inv_dt * this.m_impulse * this.m_uB.x, inv_dt * this.m_impulse * this.m_uB.y);
    }
    b2PulleyJoint.prototype.GetReactionTorque = function(inv_dt) {
        return 0.0;
    }
    b2PulleyJoint.prototype.GetGroundAnchorA = function() {
        return this.m_groundAnchorA;
    }
    b2PulleyJoint.prototype.GetGroundAnchorB = function() {
        return this.m_groundAnchorB;
    }
    b2PulleyJoint.prototype.GetLengthA = function() {
        var p = this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
        p.Subtract(this.m_groundAnchorA);
        return p.Length();
    }
    b2PulleyJoint.prototype.GetLengthB = function() {
        var p = this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
        p.Subtract(this.m_groundAnchorB);
        return p.Length();
    }
    b2PulleyJoint.prototype.GetRatio = function() {
        return this.m_ratio;
    }
    b2PulleyJoint.prototype.b2PulleyJoint = function(def) {
        this.__super.b2Joint.call(this, def);
        this.m_groundAnchorA.SetV(def.groundAnchorA);
        this.m_groundAnchorB.SetV(def.groundAnchorB);
        this.m_localAnchorA.SetV(def.localAnchorA);
        this.m_localAnchorB.SetV(def.localAnchorB);
        this.m_lengthA = def.lengthA;
        this.m_lengthB = def.lengthB;
        b2Settings.b2Assert(def.ratio != 0.0);
        this.m_ratio = def.ratio;
        this.m_constant = def.lengthA + this.m_ratio * def.lengthB;
        this.m_impulse = 0.0;
    }
    b2PulleyJoint.prototype.InitVelocityConstraints = function(data) {
        this.m_indexA = this.m_bodyA.m_islandIndex;
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_localCenterA.SetV(this.m_bodyA.m_sweep.localCenter);
        this.m_localCenterB.SetV(this.m_bodyB.m_sweep.localCenter);
        this.m_invMassA = this.m_bodyA.m_invMass;
        this.m_invMassB = this.m_bodyB.m_invMass;
        this.m_invIA = this.m_bodyA.m_invI;
        this.m_invIB = this.m_bodyB.m_invI;

        var cA = data.positions[this.m_indexA].c;
        var aA = data.positions[this.m_indexA].a;
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;

        var cB = data.positions[this.m_indexB].c;
        var aB = data.positions[this.m_indexB].a;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;

        var qA = new b2Rot(aA), qB = new b2Rot(aB);

        this.m_rA = b2Math.MulRV(qA, b2Math.SubtractVV(this.m_localAnchorA, this.m_localCenterA));
        this.m_rB = b2Math.MulRV(qB, b2Math.SubtractVV(this.m_localAnchorB, this.m_localCenterB));

        // Get the pulley axes.
        this.m_uA = b2Math.SubtractVV(b2Math.AddVV(cA, this.m_rA), this.m_groundAnchorA);
        this.m_uB = b2Math.SubtractVV(b2Math.AddVV(cB, this.m_rB), this.m_groundAnchorB);

        var lengthA = this.m_uA.Length();
        var lengthB = this.m_uB.Length();

        if (lengthA > 10.0 * b2Settings.b2_linearSlop) {
            this.m_uA.Multiply(1.0 / lengthA);
        } else {
            this.m_uA.SetZero();
        }

        if (lengthB > 10.0 * b2Settings.b2_linearSlop) {
            this.m_uB.Multiply(1.0 / lengthB);
        } else {
            this.m_uB.SetZero();
        }

        // Compute effective mass.
        var ruA = b2Math.CrossVV(this.m_rA, this.m_uA);
        var ruB = b2Math.CrossVV(this.m_rB, this.m_uB);

        var mA = this.m_invMassA + this.m_invIA * ruA * ruA;
        var mB = this.m_invMassB + this.m_invIB * ruB * ruB;

        this.m_mass = mA + this.m_ratio * this.m_ratio * mB;

        if (this.m_mass > 0.0) {
            this.m_mass = 1.0 / this.m_mass;
        }

        if (data.step.warmStarting) {
            // Scale impulses to support variable time steps.
            this.m_impulse *= data.step.dtRatio;

            // Warm starting.
            var PA = b2Math.MulFV(-(this.m_impulse), this.m_uA);
            var PB = b2Math.MulFV((-this.m_ratio * this.m_impulse), this.m_uB);

            vA.Add(b2Math.MulFV(this.m_invMassA, PA));
            wA += this.m_invIA * b2Math.CrossVV(this.m_rA, PA);
            vB.Add(b2Math.MulFV(this.m_invMassB, PB));
            wB += this.m_invIB * b2Math.CrossVV(this.m_rB, PB);
        } else {
            this.m_impulse = 0.0;
        }

//        data.velocities[this.m_indexA].v.SetV(vA);
        data.velocities[this.m_indexA].w = wA;
//        data.velocities[this.m_indexB].v.SetV(vB);
        data.velocities[this.m_indexB].w = wB;
    }
    b2PulleyJoint.prototype.SolveVelocityConstraints = function(data) {
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;

        var vpA = b2Math.AddVV(vA, b2Math.CrossFV(wA, this.m_rA));
        var vpB = b2Math.AddVV(vB, b2Math.CrossFV(wB, this.m_rB));

        var Cdot = -b2Math.DotVV(this.m_uA, vpA) - this.m_ratio * b2Math.DotVV(this.m_uB, vpB);
        var impulse = -this.m_mass * Cdot;
        this.m_impulse += impulse;

        var PA = b2Math.MulFV(-impulse, this.m_uA);
        var PB = b2Math.MulFV(-this.m_ratio * impulse, this.m_uB);
        vA.Add(b2Math.MulFV(this.m_invMassA, PA));
        wA += this.m_invIA * b2Math.CrossVV(this.m_rA, PA);
        vB.Add(b2Math.MulFV(this.m_invMassB, PB));
        wB += this.m_invIB * b2Math.CrossVV(this.m_rB, PB);

//        data.velocities[this.m_indexA].v.SetV(vA);
        data.velocities[this.m_indexA].w = wA;
//        data.velocities[this.m_indexB].v.SetV(vB);
        data.velocities[this.m_indexB].w = wB;
    }
    b2PulleyJoint.prototype.SolvePositionConstraints = function(data) {
        var cA = data.positions[this.m_indexA].c;
        var aA = data.positions[this.m_indexA].a;
        var cB = data.positions[this.m_indexB].c;
        var aB = data.positions[this.m_indexB].a;

        var qA = new b2Rot(aA), qB = new b2Rot(aB);

        var rA = b2Math.MulRV(qA, b2Math.SubtractVV(this.m_localAnchorA, this.m_localCenterA));
        var rB = b2Math.MulRV(qB, b2Math.SubtractVV(this.m_localAnchorB, this.m_localCenterB));

        // Get the pulley axes.
        var uA = b2Math.SubtractVV(b2Math.AddVV(cA, rA), this.m_groundAnchorA);
        var uB = b2Math.SubtractVV(b2Math.AddVV(cB, rB), this.m_groundAnchorB);

        var lengthA = uA.Length();
        var lengthB = uB.Length();

        if (lengthA > 10.0 * b2Settings.b2_linearSlop) {
            uA.Multiply(1.0 / lengthA);
        } else {
            uA.SetZero();
        }

        if (lengthB > 10.0 * b2Settings.b2_linearSlop) {
            uB.Multiply(1.0 / lengthB);
        } else {
            uB.SetZero();
        }

        // Compute effective mass.
        var ruA = b2Math.CrossVV(rA, uA);
        var ruB = b2Math.CrossVV(rB, uB);

        var mA = this.m_invMassA + this.m_invIA * ruA * ruA;
        var mB = this.m_invMassB + this.m_invIB * ruB * ruB;

        var mass = mA + this.m_ratio * this.m_ratio * mB;

        if (mass > 0.0) {
            mass = 1.0 / mass;
        }

        var C = this.m_constant - lengthA - this.m_ratio * lengthB;
        var linearError = b2Math.Abs(C);

        var impulse = -mass * C;

        var PA = b2Math.MulFV(-impulse, uA);
        var PB = b2Math.MulFV(-this.m_ratio * impulse, uB);

        cA.Add(b2Math.MulFV(this.m_invMassA, PA));
        aA += this.m_invIA * b2Math.CrossVV(rA, PA);
        cB.Add(b2Math.MulFV(this.m_invMassB, PB));
        aB += this.m_invIB * b2Math.CrossVV(rB, PB);

//        data.positions[this.m_indexA].c.SetV(cA);
        data.positions[this.m_indexA].a = aA;
//        data.positions[this.m_indexB].c.SetV(cB);
        data.positions[this.m_indexB].a = aB;

        return linearError < b2Settings.b2_linearSlop;
    }
    Box2D.inherit(b2PulleyJointDef, Box2D.Dynamics.Joints.b2JointDef);
    b2PulleyJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
    b2PulleyJointDef.b2PulleyJointDef = function() {
        Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
        this.groundAnchorA = new b2Vec2();
        this.groundAnchorB = new b2Vec2();
        this.localAnchorA = new b2Vec2();
        this.localAnchorB = new b2Vec2();
    };
    b2PulleyJointDef.prototype.b2PulleyJointDef = function() {
        this.__super.b2JointDef.call(this);
        this.type = b2Joint.e_pulleyJoint;
        this.groundAnchorA.Set((-1.0), 1.0);
        this.groundAnchorB.Set(1.0, 1.0);
        this.localAnchorA.Set((-1.0), 0.0);
        this.localAnchorB.Set(1.0, 0.0);
        this.lengthA = 0.0;
        this.lengthB = 0.0;
        this.ratio = 1.0;
        this.collideConnected = true;
    }
    b2PulleyJointDef.prototype.Initialize = function(bA, bB, gaA, gaB, anchorA, anchorB, r) {
        this.bodyA = bA;
        this.bodyB = bB;
        this.groundAnchorA.SetV(gaA);
        this.groundAnchorB.SetV(gaB);
        this.localAnchorA = this.bodyA.GetLocalPoint(anchorA);
        this.localAnchorB = this.bodyB.GetLocalPoint(anchorB);
        var d1X = anchorA.x - gaA.x;
        var d1Y = anchorA.y - gaA.y;
        this.lengthA = Math.sqrt(d1X * d1X + d1Y * d1Y);
        var d2X = anchorB.x - gaB.x;
        var d2Y = anchorB.y - gaB.y;
        this.lengthB = Math.sqrt(d2X * d2X + d2Y * d2Y);
        this.ratio = r;
    }
    Box2D.inherit(b2RevoluteJoint, Box2D.Dynamics.Joints.b2Joint);
    b2RevoluteJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
    b2RevoluteJoint.b2RevoluteJoint = function() {
        Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
        this.m_localAnchorA = new b2Vec2();
        this.m_localAnchorB = new b2Vec2();
        this.m_impulse = new b2Vec3();
        this.m_rA = new b2Vec2();
        this.m_rB = new b2Vec2();
        this.m_localCenterA = new b2Vec2();
        this.m_localCenterB = new b2Vec2();
        this.m_mass = new b2Mat33();
    };
    b2RevoluteJoint.prototype.GetLocalAnchorA = function() {
        return this.m_localAnchorA;
    }
    b2RevoluteJoint.prototype.GetLocalAnchorB = function() {
        return this.m_localAnchorB;
    }
    b2RevoluteJoint.prototype.GetAnchorA = function() {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
    }
    b2RevoluteJoint.prototype.GetAnchorB = function() {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
    }
    b2RevoluteJoint.prototype.GetReactionForce = function(inv_dt) {
        if (inv_dt === undefined) inv_dt = 0;
        return new b2Vec2(inv_dt * this.m_impulse.x, inv_dt * this.m_impulse.y);
    }
    b2RevoluteJoint.prototype.GetReactionTorque = function(inv_dt) {
        if (inv_dt === undefined) inv_dt = 0;
        return inv_dt * this.m_impulse.z;
    }
    b2RevoluteJoint.prototype.GetJointAngle = function() {
        return this.m_bodyB.m_sweep.a - this.m_bodyA.m_sweep.a - this.m_referenceAngle;
    }
    b2RevoluteJoint.prototype.GetJointSpeed = function() {
        return this.m_bodyB.m_angularVelocity - this.m_bodyA.m_angularVelocity;
    }
    b2RevoluteJoint.prototype.IsLimitEnabled = function() {
        return this.m_enableLimit;
    }
    b2RevoluteJoint.prototype.EnableLimit = function(flag) {
        this.m_enableLimit = flag;
    }
    b2RevoluteJoint.prototype.GetLowerLimit = function() {
        return this.m_lowerAngle;
    }
    b2RevoluteJoint.prototype.GetUpperLimit = function() {
        return this.m_upperAngle;
    }
    b2RevoluteJoint.prototype.SetLimits = function(lower, upper) {
        if (lower === undefined) lower = 0;
        if (upper === undefined) upper = 0;
        this.m_lowerAngle = lower;
        this.m_upperAngle = upper;
    }
    b2RevoluteJoint.prototype.IsMotorEnabled = function() {
        this.m_bodyA.SetAwake(true);
        this.m_bodyB.SetAwake(true);
        return this.m_enableMotor;
    }
    b2RevoluteJoint.prototype.EnableMotor = function(flag) {
        this.m_enableMotor = flag;
    }
    b2RevoluteJoint.prototype.SetMotorSpeed = function(speed) {
        if (speed === undefined) speed = 0;
        this.m_bodyA.SetAwake(true);
        this.m_bodyB.SetAwake(true);
        this.m_motorSpeed = speed;
    }
    b2RevoluteJoint.prototype.GetMotorSpeed = function() {
        return this.m_motorSpeed;
    }
    b2RevoluteJoint.prototype.SetMaxMotorTorque = function(torque) {
        if (torque === undefined) torque = 0;
        this.m_maxMotorTorque = torque;
    }
    b2RevoluteJoint.prototype.GetMaxMotorTorque = function() {
        return this.m_maxMotorTorque;
    }
    b2RevoluteJoint.prototype.GetMotorTorque = function(inv_dt) {
        return inv_dt * this.m_motorImpulse;
    }
    b2RevoluteJoint.prototype.b2RevoluteJoint = function(def) {
        this.__super.b2Joint.call(this, def);
        this.m_localAnchorA.SetV(def.localAnchorA);
        this.m_localAnchorB.SetV(def.localAnchorB);
        this.m_referenceAngle = def.referenceAngle;
 
        this.m_impulse.SetZero();
        this.m_motorImpulse = 0.0;
 
        this.m_lowerAngle = def.lowerAngle;
        this.m_upperAngle = def.upperAngle;
        this.m_maxMotorTorque = def.maxMotorTorque;
        this.m_motorSpeed = def.motorSpeed;
        this.m_enableLimit = def.enableLimit;
        this.m_enableMotor = def.enableMotor;
        this.m_limitState = b2Joint.e_inactiveLimit;
    }
    b2RevoluteJoint.prototype.InitVelocityConstraints = function(data) {
        this.m_indexA = this.m_bodyA.m_islandIndex;
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_localCenterA.SetV(this.m_bodyA.m_sweep.localCenter);
        this.m_localCenterB.SetV(this.m_bodyB.m_sweep.localCenter);
        this.m_invMassA = this.m_bodyA.m_invMass;
        this.m_invMassB = this.m_bodyB.m_invMass;
        this.m_invIA = this.m_bodyA.m_invI;
        this.m_invIB = this.m_bodyB.m_invI;

        var cA = data.positions[this.m_indexA].c;
        var aA = data.positions[this.m_indexA].a;
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;

        var cB = data.positions[this.m_indexB].c;
        var aB = data.positions[this.m_indexB].a;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;

        var qA = new b2Rot(aA), qB = new b2Rot(aB);

        var x = this.m_localAnchorA.x - this.m_localCenterA.x;
        var y = this.m_localAnchorA.y - this.m_localCenterA.y;
        this.m_rA.x = qA.c * x - qA.s * y;
        this.m_rA.y = qA.s * x + qA.c * y;
        x = this.m_localAnchorB.x - this.m_localCenterB.x;
        y = this.m_localAnchorB.y - this.m_localCenterB.y;
        this.m_rB.x = qB.c * x - qB.s * y;
        this.m_rB.y = qB.s * x + qB.c * y;

        // J = [-I -r1_skew I r2_skew]
        //     [ 0       -1 0       1]
        // r_skew = [-ry; rx]

        // Matlab
        // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
        //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
        //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

        var mA = this.m_invMassA, mB = this.m_invMassB;
        var iA = this.m_invIA, iB = this.m_invIB;

        var fixedRotation = (iA + iB == 0.0);

        this.m_mass.ex.x = mA + mB + this.m_rA.y * this.m_rA.y * iA + this.m_rB.y * this.m_rB.y * iB;
        this.m_mass.ey.x = -this.m_rA.y * this.m_rA.x * iA - this.m_rB.y * this.m_rB.x * iB;
        this.m_mass.ez.x = -this.m_rA.y * iA - this.m_rB.y * iB;
        this.m_mass.ex.y = this.m_mass.ey.x;
        this.m_mass.ey.y = mA + mB + this.m_rA.x * this.m_rA.x * iA + this.m_rB.x * this.m_rB.x * iB;
        this.m_mass.ez.y = this.m_rA.x * iA + this.m_rB.x * iB;
        this.m_mass.ex.z = this.m_mass.ez.x;
        this.m_mass.ey.z = this.m_mass.ez.y;
        this.m_mass.ez.z = iA + iB;

        this.m_motorMass = iA + iB;
        if (this.m_motorMass > 0.0) {
            this.m_motorMass = 1.0 / this.m_motorMass;
        }

        if (this.m_enableMotor == false || fixedRotation) {
           this.m_motorImpulse = 0.0;
        }

        if (b2Joint.m_enableLimit && fixedRotation == false) {
            var jointAngle = aB - aA - this.m_referenceAngle;
            if (b2Math.Abs(this.m_upperAngle - this.m_lowerAngle) < 2.0 * b2Settings.b2_angularSlop) {
                this.m_limitState = b2Joint.e_equalLimits;
            } else if (jointAngle <= this.m_lowerAngle) {
                if (this.m_limitState != b2Joint.e_atLowerLimit) {
                    this.m_impulse.z = 0.0;
                }
                this.m_limitState = b2Joint.e_atLowerLimit;
            } else if (jointAngle >= this.m_upperAngle) {
                if (this.m_limitState != b2Joint.e_atUpperLimit) {
                    this.m_impulse.z = 0.0;
                }
                this.m_limitState = b2Joint.e_atUpperLimit;
            } else {
                this.m_limitState = b2Joint.e_inactiveLimit;
                this.m_impulse.z = 0.0;
            }
        } else {
            this.m_limitState = b2Joint.e_inactiveLimit;
        }

        if (data.step.warmStarting) {
            // Scale impulses to support a variable time step.
            this.m_impulse.Multiply(data.step.dtRatio);
            this.m_motorImpulse *= data.step.dtRatio;

            var P = new b2Vec2(this.m_impulse.x, this.m_impulse.y);

            vA.Subtract(b2Math.MulFV(mA, P));
            wA -= iA * (b2Math.CrossVV(this.m_rA, P) + this.m_motorImpulse + this.m_impulse.z);

            vB.Add(b2Math.MulFV(mB, P));
            wB += iB * (b2Math.CrossVV(this.m_rB, P) + this.m_motorImpulse + this.m_impulse.z);
        } else {
            this.m_impulse.SetZero();
            this.m_motorImpulse = 0.0;
        }

//        data.velocities[this.m_indexA].v = vA.Copy();
        data.velocities[this.m_indexA].w = wA;
//        data.velocities[this.m_indexB].v = vB.Copy();
        data.velocities[this.m_indexB].w = wB;
    }
    b2RevoluteJoint.prototype.SolveVelocityConstraints = function(data) {
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;

        var mA = this.m_invMassA, mB = this.m_invMassB;
        var iA = this.m_invIA, iB = this.m_invIB;

        var fixedRotation = (iA + iB == 0.0);

        // Solve motor constraint.
        if (this.m_enableMotor && this.m_limitState != b2Joint.e_equalLimits && fixedRotation == false) {
            var Cdot = wB - wA - this.m_motorSpeed;
            var impulse = -this.m_motorMass * Cdot;
            var oldImpulse = this.m_motorImpulse;
            var maxImpulse = data.step.dt * this.m_maxMotorTorque;
            this.m_motorImpulse = b2Math.Clamp(this.m_motorImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = this.m_motorImpulse - oldImpulse;

            wA -= iA * impulse;
            wB += iB * impulse;
        }

        // Solve limit constraint.
        if (this.m_enableLimit && this.m_limitState != b2Joint.e_inactiveLimit && fixedRotation == false) {
            var Cdot1 = vB.Copy(); Cdot1.Add(b2Math.CrossFV(wB, this.m_rB)); Cdot1.Subtract(vA); Cdot1.Subtract(b2Math.CrossFV(wA, this.m_rA));
            var Cdot2 = wB - wA;
            var Cdot = new b2Vec3(Cdot1.x, Cdot1.y, Cdot2);

            var impulse = this.m_mass.Solve33(Cdot).GetNegative();

            if (this.m_limitState == b2Joint.e_equalLimits) {
                this.m_impulse.Add(impulse);
            } else if (this.m_limitState == b2Joint.e_atLowerLimit) {
                var newImpulse = this.m_impulse.z + impulse.z;
                if (newImpulse < 0.0) {
                    var rhs = Cdot1.GetNegative(); rhs.Add(b2Math.MulFV(this.m_impulse.z, new b2Vec2(this.m_mass.ez.x, this.m_mass.ez.y)));
                    var reduced = this.m_mass.Solve22(rhs);
                    impulse.x = reduced.x;
                    impulse.y = reduced.y;
                    impulse.z = -this.m_impulse.z;
                    this.m_impulse.x += reduced.x;
                    this.m_impulse.y += reduced.y;
                    this.m_impulse.z = 0.0;
                } else {
                    this.m_impulse.Add(impulse);
                }
            } else if (this.m_limitState == b2Joint.e_atUpperLimit) {
                var newImpulse = this.m_impulse.z + impulse.z;
                if (newImpulse > 0.0) {
                    var rhs = Cdot1.GetNegative(); rhs.Add(b2Math.MulFV(this.m_impulse.z, new b2Vec2(this.m_mass.ez.x, this.m_mass.ez.y)));
                    var reduced = this.m_mass.Solve22(rhs);
                    impulse.x = reduced.x;
                    impulse.y = reduced.y;
                    impulse.z = -this.m_impulse.z;
                    this.m_impulse.x += reduced.x;
                    this.m_impulse.y += reduced.y;
                    this.m_impulse.z = 0.0;
                } else {
                    this.m_impulse.Add(impulse);
                }
            }

            var P = new b2Vec2(impulse.x, impulse.y);

            vA.Subtract(b2Math.MulFV(mA, P));
            wA -= iA * (b2Math.CrossVV(this.m_rA, P) + impulse.z);

            vB.Add(b2Math.MulFV(mB, P));
            wB += iB * (b2Math.CrossVV(this.m_rB, P) + impulse.z);
        } else {
            // Solve point-to-point constraint
            var Cdot = vB.Copy(); Cdot.Add(b2Math.CrossFV(wB, this.m_rB)); Cdot.Subtract(vA); Cdot.Subtract(b2Math.CrossFV(wA, this.m_rA));
            var impulse = this.m_mass.Solve22(Cdot.GetNegative());

            this.m_impulse.x += impulse.x;
            this.m_impulse.y += impulse.y;

            vA.Subtract(b2Math.MulFV(mA, impulse));
            wA -= iA * b2Math.CrossVV(this.m_rA, impulse);

            vB.Add(b2Math.MulFV(mB, impulse));
            wB += iB * b2Math.CrossVV(this.m_rB, impulse);
        }

//        data.velocities[this.m_indexA].v = vA.Copy();
        data.velocities[this.m_indexA].w = wA;
//        data.velocities[this.m_indexB].v = vB.Copy();
        data.velocities[this.m_indexB].w = wB;
    }
    b2RevoluteJoint.prototype.SolvePositionConstraints = function(data) {
        var cA = data.positions[this.m_indexA].c;
        var aA = data.positions[this.m_indexA].a;
        var cB = data.positions[this.m_indexB].c;
        var aB = data.positions[this.m_indexB].a;

        var qA = new b2Rot(aA), qB = new b2Rot(aB);

        var angularError = 0.0;
        var positionError = 0.0;

        var fixedRotation = (this.m_invIA + this.m_invIB == 0.0);

        // Solve angular limit constraint.
        if (this.m_enableLimit && this.m_limitState != b2Joint.e_inactiveLimit && fixedRotation == false) {
            var angle = aB - aA - this.m_referenceAngle;
            var limitImpulse = 0.0;

            if (this.m_limitState == b2Joint.e_equalLimits) {
                // Prevent large angular corrections
                var C = b2Math.Clamp(angle - this.m_lowerAngle, -b2Settings.b2_maxAngularCorrection, b2Settings.b2_maxAngularCorrection);
                limitImpulse = -this.m_motorMass * C;
                angularError = b2Math.Abs(C);
            } else if (this.m_limitState == b2Joint.e_atLowerLimit) {
                var C = angle - this.m_lowerAngle;
                angularError = -C;

                // Prevent large angular corrections and allow some slop.
                C = b2Math.Clamp(C + b2Settings.b2_angularSlop, -b2Settings.b2_maxAngularCorrection, 0.0);
                limitImpulse = -this.m_motorMass * C;
            } else if (this.m_limitState == b2Joint.e_atUpperLimit) {
                var C = angle - this.m_upperAngle;
                angularError = C;

                // Prevent large angular corrections and allow some slop.
                C = b2Math.Clamp(C - b2Settings.b2_angularSlop, 0.0, b2Settings.b2_maxAngularCorrection);
                limitImpulse = -this.m_motorMass * C;
            }

            aA -= this.m_invIA * limitImpulse;
            aB += this.m_invIB * limitImpulse;
        }

        // Solve point-to-point constraint.
        {
            qA.Set(aA);
            qB.Set(aB);
            var x = this.m_localAnchorA.x - this.m_localCenterA.x;
            var y = this.m_localAnchorA.y - this.m_localCenterA.y;
            var rA = new b2Vec2(qA.c * x - qA.s * y, qA.s * x + qA.c * y);
            x = this.m_localAnchorB.x - this.m_localCenterB.x;
            y = this.m_localAnchorB.y - this.m_localCenterB.y;
            var rB = new b2Vec2(qB.c * x - qB.s * y, qB.s * x + qB.c * y);;

            var C = cB.Copy(); C.Add(rB); C.Subtract(cA); C.Subtract(rA);
            positionError = C.Length();

            var mA = this.m_invMassA, mB = this.m_invMassB;
            var iA = this.m_invIA, iB = this.m_invIB;

            var K = new b2Mat22();
            K.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y;
            K.ex.y = -iA * rA.x * rA.y - iB * rB.x * rB.y;
            K.ey.x = K.ex.y;
            K.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x;

            var impulse = K.Solve(C).GetNegative();

            cA.Subtract(b2Math.MulFV(mA, impulse));
            aA -= iA * b2Math.CrossVV(rA, impulse);

            cB.Add(b2Math.MulFV(mB, impulse));
            aB += iB * b2Math.CrossVV(rB, impulse);
        }

//        data.positions[this.m_indexA].c = cA.Copy();
        data.positions[this.m_indexA].a = aA;
//        data.positions[this.m_indexB].c = cB.Copy();
        data.positions[this.m_indexB].a = aB;

        return positionError <= b2Settings.b2_linearSlop && angularError <= b2Settings.b2_angularSlop;
    }
    Box2D.postDefs.push(function() {
        Box2D.Dynamics.Joints.b2RevoluteJoint.tImpulse = new b2Vec2();
    });
    Box2D.inherit(b2RevoluteJointDef, Box2D.Dynamics.Joints.b2JointDef);
    b2RevoluteJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
    b2RevoluteJointDef.b2RevoluteJointDef = function() {
        Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
        this.localAnchorA = new b2Vec2();
        this.localAnchorB = new b2Vec2();
    };
    b2RevoluteJointDef.prototype.b2RevoluteJointDef = function() {
        this.__super.b2JointDef.call(this);
        this.type = b2Joint.e_revoluteJoint;
        this.localAnchorA.Set(0.0, 0.0);
        this.localAnchorB.Set(0.0, 0.0);
        this.referenceAngle = 0.0;
        this.lowerAngle = 0.0;
        this.upperAngle = 0.0;
        this.maxMotorTorque = 0.0;
        this.motorSpeed = 0.0;
        this.enableLimit = false;
        this.enableMotor = false;
    }
    b2RevoluteJointDef.prototype.Initialize = function(bA, bB, anchor) {
        this.bodyA = bA;
        this.bodyB = bB;
        this.localAnchorA = this.bodyA.GetLocalPoint(anchor);
        this.localAnchorB = this.bodyB.GetLocalPoint(anchor);
        this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
    }
    Box2D.inherit(b2RopeJoint, Box2D.Dynamics.Joints.b2Joint);
    b2RopeJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
    b2RopeJoint.b2RopeJoint = function() {
        Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
        this.m_localAnchorA = new b2Vec2();
        this.m_localAnchorB = new b2Vec2();
        this.m_u = new b2Vec2();
        this.m_rA = new b2Vec2();
        this.m_rB = new b2Vec2();
        this.m_localCenterA = new b2Vec2();
        this.m_localCenterB = new b2Vec2();
    };
    b2RopeJoint.prototype.GetLocalAnchorA = function() {
        return this.m_localAnchorA;
    }
    b2RopeJoint.prototype.GetLocalAnchorB = function() {
        return this.m_localAnchorB;
    }
    b2RopeJoint.prototype.GetMaxLength = function() {
        return this.m_maxLength;
    }
    b2RopeJoint.prototype.SetMaxLength = function(len) {
        this.m_maxLength = len;
    }
    b2RopeJoint.prototype.GetLimitState = function(len) {
        return this.m_state;
    }
    b2RopeJoint.prototype.GetAnchorA = function() {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
    }
    b2RopeJoint.prototype.GetAnchorB = function() {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
    }
    b2RopeJoint.prototype.GetReactionForce = function(inv_dt) {
        return new b2Vec2(inv_dt * this.m_impulse * this.m_u.x, inv_dt * this.m_impulse * this.m_u.y);
    }
    b2RopeJoint.prototype.GetReactionTorque = function(inv_dt) {
        return 0.0;
    }
    b2RopeJoint.prototype.b2RopeJoint = function(def) {
        this.__super.b2Joint.call(this, def);
        this.m_localAnchorA.SetV(def.localAnchorA);
        this.m_localAnchorB.SetV(def.localAnchorB);
        this.m_maxLength = def.maxLength;

        this.m_mass = 0.0;
        this.m_impulse = 0.0;
        this.m_state = b2Joint.e_inactiveLimit;
        this.m_length = 0.0;
    }
    b2RopeJoint.prototype.InitVelocityConstraints = function(data) {
        this.m_indexA = this.m_bodyA.m_islandIndex;
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_localCenterA.SetV(this.m_bodyA.m_sweep.localCenter);
        this.m_localCenterB.SetV(this.m_bodyB.m_sweep.localCenter);
        this.m_invMassA = this.m_bodyA.m_invMass;
        this.m_invMassB = this.m_bodyB.m_invMass;
        this.m_invIA = this.m_bodyA.m_invI;
        this.m_invIB = this.m_bodyB.m_invI;

        var cA = data.positions[this.m_indexA].c;
        var aA = data.positions[this.m_indexA].a;
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;

        var cB = data.positions[this.m_indexB].c;
        var aB = data.positions[this.m_indexB].a;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;

        var qA = new b2Rot(aA);
        var qB = new b2Rot(aB);

        this.m_rA = b2Math.MulRV(qA, b2Math.SubtractVV(this.m_localAnchorA, this.m_localCenterA));
        this.m_rB = b2Math.MulRV(qB, b2Math.SubtractVV(this.m_localAnchorB, this.m_localCenterB));
        this.m_u = b2Math.AddVV(cB, b2Math.SubtractVV(b2Math.SubtractVV(this.m_rB, cA), this.m_rA));

        this.m_length = this.m_u.Length();

        var C = this.m_length - this.m_maxLength;
        if (C > 0.0) {
            this.m_state = b2Joint.e_atUpperLimit;
        } else {
            this.m_state = b2Joint.e_inactiveLimit;
        }

        if (this.m_length > b2Settings.b2_linearSlop) {
            this.m_u.Multiply(1.0 / this.m_length);
        } else {
            this.m_u.SetZero();
            this.m_mass = 0.0;
            this.m_impulse = 0.0;
            return;
        }

        // Compute effective mass.
        var crA = b2Math.CrossVV(this.m_rA, this.m_u);
        var crB = b2Math.CrossVV(this.m_rB, this.m_u);
        var invMass = this.m_invMassA + this.m_invIA * crA * crA + this.m_invMassB + this.m_invIB * crB * crB;

        this.m_mass = (invMass != 0.0) ? 1.0 / invMass : 0.0;

        if (data.step.warmStarting) {
            // Scale the impulse to support a variable time step.
            this.m_impulse *= data.step.dtRatio;

            var P = this.m_u.Copy();
            P.Multiply(this.m_impulse);
            var tmp = P.Copy();
            tmp.Multiply(this.m_invMassA);
            vA.Subtract(tmp);
            wA -= this.m_invIA * b2Math.CrossVV(this.m_rA, P);
            tmp.SetV(P);
            tmp.Multiply(this.m_invMassB);
            vB.Add(tmp);
            wB += this.m_invIB * b2Math.CrossVV(this.m_rB, P);
        } else {
            this.m_impulse = 0.0;
        }

//        data.velocities[this.m_indexA].v.SetV(vA);
        data.velocities[this.m_indexA].w = wA;
//        data.velocities[this.m_indexB].v.SetV(vB);
        data.velocities[this.m_indexB].w = wB;
    }
    b2RopeJoint.prototype.SolveVelocityConstraints = function(data) {
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;

        var vpA = b2Math.AddVV(vA, b2Math.CrossFV(wA, this.m_rA));
        var vpB = b2Math.AddVV(vB, b2Math.CrossFV(wB, this.m_rB));
        var C = this.m_length - this.m_maxLength;
        var Cdot = b2Math.DotVV(this.m_u, b2Math.SubtractVV(vpB, vpA));

        // Predictive constraint.
        if (C < 0.0) {
            Cdot += data.step.inv_dt * C;
        }

        var impulse = -this.m_mass * Cdot;
        var oldImpulse = this.m_impulse;
        this.m_impulse = b2Math.Min(0.0, this.m_impulse + impulse);
        impulse = this.m_impulse - oldImpulse;

        var P = this.m_u.Copy();
        P.Multiply(impulse);
        var tmp = P.Copy();
        tmp.Multiply(this.m_invMassA);
        vA.Subtract(tmp);
        wA -= this.m_invIA * b2Math.CrossVV(this.m_rA, P);
        tmp.SetV(P);
        tmp.Multiply(this.m_invMassB);
        vB.Add(tmp);
        wB += this.m_invIB * b2Math.CrossVV(this.m_rB, P);

//        data.velocities[this.m_indexA].v.SetV(vA);
        data.velocities[this.m_indexA].w = wA;
//        data.velocities[this.m_indexB].v.SetV(vB);
        data.velocities[this.m_indexB].w = wB;
    }
    b2RopeJoint.prototype.SolvePositionConstraints = function(data) {
        var cA = data.positions[this.m_indexA].c;
        var aA = data.positions[this.m_indexA].a;
        var cB = data.positions[this.m_indexB].c;
        var aB = data.positions[this.m_indexB].a;

        var qA = new b2Rot(aA);
        var qB = new b2Rot(aB);

        var rA = b2Math.MulRV(qA, b2Math.SubtractVV(this.m_localAnchorA, this.m_localCenterA));
        var rB = b2Math.MulRV(qB, b2Math.SubtractVV(this.m_localAnchorB, this.m_localCenterB));
        var u = b2Math.SubtractVV(b2Math.SubtractVV(b2Math.AddVV(cB, rB), cA), rA);

        var length = u.Normalize();
        var C = length - this.m_maxLength;

        C = b2Math.Clamp(C, 0.0, b2Settings.b2_maxLinearCorrection);

        var impulse = -this.m_mass * C;
        var P = u.Copy();
        P.Multiply(impulse);

        var tmp = P.Copy();
        tmp.Multiply(this.m_invMassA);
        cA.Subtract(tmp);
        aA -= this.m_invIA * b2Math.CrossVV(rA, P);
        tmp.SetV(P);
        tmp.Multiply(this.m_invMassB);
        cB.Add(tmp);
        aB += this.m_invIB * b2Math.CrossVV(rB, P);

//        data.positions[this.m_indexA].c = cA.Copy();
        data.positions[this.m_indexA].a = aA;
//        data.positions[this.m_indexB].c = cB.Copy();
        data.positions[this.m_indexB].a = aB;

        return length - this.m_maxLength < b2Settings.b2_linearSlop;
    }
    Box2D.inherit(b2RopeJointDef, Box2D.Dynamics.Joints.b2JointDef);
    b2RopeJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
    b2RopeJointDef.b2RopeJointDef = function() {
        Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
        this.localAnchorA = new b2Vec2();
        this.localAnchorB = new b2Vec2();
    }
    b2RopeJointDef.prototype.b2RopeJointDef = function() {
        this.__super.b2JointDef.call(this);
        this.type = b2Joint.e_ropeJoint;
        this.localAnchorA.Set(-1.0, 0.0);
        this.localAnchorB.Set(1.0, 0.0);
        this.maxLength = 0.0;
    }
    Box2D.inherit(b2WeldJoint, Box2D.Dynamics.Joints.b2Joint);
    b2WeldJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
    b2WeldJoint.b2WeldJoint = function() {
        Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
        this.m_localAnchorA = new b2Vec2();
        this.m_localAnchorB = new b2Vec2();
        this.m_impulse = new b2Vec3();
        this.m_rA = new b2Vec2();
        this.m_rB = new b2Vec2();
        this.m_localCenterA = new b2Vec2();
        this.m_localCenterB = new b2Vec2();
        this.m_mass = new b2Mat33();
    };
    b2WeldJoint.prototype.GetLocalAnchorA = function() {
        return this.m_localAnchorA;
    }
    b2WeldJoint.prototype.GetLocalAnchorB = function() {
        return this.m_localAnchorB;
    }
    b2WeldJoint.prototype.GetReferenceAngle = function() {
        return this.m_referenceAngle;
    }
    b2WeldJoint.prototype.GetFrequency = function() {
        return this.m_frequencyHz;
    }
    b2WeldJoint.prototype.SetFrequency = function(value) {
        this.m_frequencyHz = value;
    }
    b2WeldJoint.prototype.GetDampingRatio = function() {
        return this.m_dampingRatio;
    }
    b2WeldJoint.prototype.SetDampingRatio = function(value) {
        this.m_dampingRatio = value;
    }
    b2WeldJoint.prototype.GetAnchorA = function() {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
    }
    b2WeldJoint.prototype.GetAnchorB = function() {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
    }
    b2WeldJoint.prototype.GetReactionForce = function(inv_dt) {
        return new b2Vec2(inv_dt * this.m_impulse.x, inv_dt * this.m_impulse.y);
    }
    b2WeldJoint.prototype.GetReactionTorque = function(inv_dt) {
        return inv_dt * this.m_impulse.z;
    }
    b2WeldJoint.prototype.b2WeldJoint = function(def) {
        this.__super.b2Joint.call(this, def);
        this.m_localAnchorA.SetV(def.localAnchorA);
        this.m_localAnchorB.SetV(def.localAnchorB);
        this.m_referenceAngle = def.referenceAngle;
        this.m_frequencyHz = def.frequencyHz;
        this.m_dampingRatio = def.dampingRatio;
        this.m_impulse.SetZero();
    }
    b2WeldJoint.prototype.InitVelocityConstraints = function(data) {
        this.m_indexA = this.m_bodyA.m_islandIndex;
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_localCenterA.SetV(this.m_bodyA.m_sweep.localCenter);
        this.m_localCenterB.SetV(this.m_bodyB.m_sweep.localCenter);
        this.m_invMassA = this.m_bodyA.m_invMass;
        this.m_invMassB = this.m_bodyB.m_invMass;
        this.m_invIA = this.m_bodyA.m_invI;
        this.m_invIB = this.m_bodyB.m_invI;

        var cA = data.positions[this.m_indexA].c;
        var aA = data.positions[this.m_indexA].a;
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;

        var cB = data.positions[this.m_indexB].c;
        var aB = data.positions[this.m_indexB].a;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;

        var qA = new b2Rot(aA), qB = new b2Rot(aB);

        this.m_rA = b2Math.MulRV(qA, b2Math.SubtractVV(this.m_localAnchorA, this.m_localCenterA));
        this.m_rB = b2Math.MulRV(qB, b2Math.SubtractVV(this.m_localAnchorB, this.m_localCenterB));

        // J = [-I -r1_skew I r2_skew]
        //     [ 0       -1 0       1]
        // r_skew = [-ry; rx]

        // Matlab
        // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
        //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
        //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

        var mA = this.m_invMassA, mB = this.m_invMassB;
        var iA = this.m_invIA, iB = this.m_invIB;

        var K = new b2Mat33();
        K.ex.x = mA + mB + this.m_rA.y * this.m_rA.y * iA + this.m_rB.y * this.m_rB.y * iB;
        K.ey.x = -this.m_rA.y * this.m_rA.x * iA - this.m_rB.y * this.m_rB.x * iB;
        K.ez.x = -this.m_rA.y * iA - this.m_rB.y * iB;
        K.ex.y = K.ey.x;
        K.ey.y = mA + mB + this.m_rA.x * this.m_rA.x * iA + this.m_rB.x * this.m_rB.x * iB;
        K.ez.y = this.m_rA.x * iA + this.m_rB.x * iB;
        K.ex.z = K.ez.x;
        K.ey.z = K.ez.y;
        K.ez.z = iA + iB;

        if (this.m_frequencyHz > 0.0) {
            K.GetInverse22(this.m_mass);

            var invM = iA + iB;
            var m = invM > 0.0 ? 1.0 / invM : 0.0;

            var C = aB - aA - this.m_referenceAngle;

            // Frequency
            var omega = 2.0 * b2Settings.b2_pi * this.m_frequencyHz;

            // Damping coefficient
            var d = 2.0 * m * this.m_dampingRatio * omega;

            // Spring stiffness
            var k = m * omega * omega;

            // magic formulas
            var h = data.step.dt;
            this.m_gamma = h * (d + h * k);
            this.m_gamma = this.m_gamma != 0.0 ? 1.0 / this.m_gamma : 0.0;
            this.m_bias = C * h * k * this.m_gamma;

            invM += this.m_gamma;
            this.m_mass.ez.z = invM != 0.0 ? 1.0 / invM : 0.0;
        } else {
            K.GetSymInverse33(this.m_mass);
            this.m_gamma = 0.0;
            this.m_bias = 0.0;
        }

        if (data.step.warmStarting) {
            // Scale impulses to support a variable time step.
            this.m_impulse.Multiply(data.step.dtRatio);

            var P = new b2Vec2(this.m_impulse.x, this.m_impulse.y);

            vA.Subtract(b2Math.MulFV(mA, P));
            wA -= iA * (b2Math.CrossVV(this.m_rA, P) + this.m_impulse.z);

            vB.Add(b2Math.MulFV(mB, P));
            wB += iB * (b2Math.CrossVV(this.m_rB, P) + this.m_impulse.z);
        } else {
            this.m_impulse.SetZero();
        }

//        data.velocities[this.m_indexA].v.SetV(vA);
        data.velocities[this.m_indexA].w = wA;
//        data.velocities[this.m_indexB].v.SetV(vB);
        data.velocities[this.m_indexB].w = wB;
    }
    b2WeldJoint.prototype.SolveVelocityConstraints = function(data) {
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;

        var mA = this.m_invMassA, mB = this.m_invMassB;
        var iA = this.m_invIA, iB = this.m_invIB;

        if (this.m_frequencyHz > 0.0) {
            var Cdot2 = wB - wA;

            var impulse2 = -this.m_mass.ez.z * (Cdot2 + this.m_bias + this.m_gamma * this.m_impulse.z);
            this.m_impulse.z += impulse2;

            wA -= iA * impulse2;
            wB += iB * impulse2;

            var Cdot1 = b2Math.AddVV(vB, b2Math.CrossFV(wB, this.m_rB));
            Cdot1.Subtract(vA); Cdot1.Subtract(b2Math.CrossFV(wA, this.m_rA));

            var impulse1 = b2Math.MulMV(this.m_mass, Cdot1).GetNegative();
            this.m_impulse.x += impulse1.x;
            this.m_impulse.y += impulse1.y;

            var P = impulse1.Copy();

            vA.Subtract(b2Math.MulFV(mA, P));
            wA -= iA * b2Math.CrossVV(this.m_rA, P);

            vB.Add(b2Math.MulFV(mB, P));
            wB += iB * b2Math.CrossVV(this.m_rB, P);
        } else {
            var Cdot1 = b2Math.AddVV(vB, b2Math.CrossFV(wB, this.m_rB));
            Cdot1.Subtract(vA); Cdot1.Subtract(b2Math.CrossFV(wA, this.m_rA));
            var Cdot2 = wB - wA;
            var Cdot = new b2Vec3(Cdot1.x, Cdot1.y, Cdot2);

            var impulse = b2Math.MulMV3(this.m_mass, Cdot).GetNegative();
            this.m_impulse.Add(impulse);

            var P = new b2Vec2(impulse.x, impulse.y);

            vA.Subtract(b2Math.MulFV(mA, P));
            wA -= iA * (b2Math.CrossVV(this.m_rA, P) + impulse.z);

            vB.Add(b2Math.MulFV(mB, P));
            wB += iB * (b2Math.CrossVV(this.m_rB, P) + impulse.z);
        }

//        data.velocities[this.m_indexA].v.SetV(vA);
        data.velocities[this.m_indexA].w = wA;
//        data.velocities[this.m_indexB].v.SetV(vB);
        data.velocities[this.m_indexB].w = wB;
    }
    b2WeldJoint.prototype.SolvePositionConstraints = function(data) {
        var cA = data.positions[this.m_indexA].c;
        var aA = data.positions[this.m_indexA].a;
        var cB = data.positions[this.m_indexB].c;
        var aB = data.positions[this.m_indexB].a;

        var qA = new b2Rot(aA), qB = new b2Rot(aB);

        var mA = this.m_invMassA, mB = this.m_invMassB;
        var iA = this.m_invIA, iB = this.m_invIB;

        var rA = b2Math.MulRV(qA, b2Math.SubtractVV(this.m_localAnchorA, this.m_localCenterA));
        var rB = b2Math.MulRV(qB, b2Math.SubtractVV(this.m_localAnchorB, this.m_localCenterB));

        var positionError, angularError;

        var K = new b2Mat33();
        K.ex.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
        K.ey.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
        K.ez.x = -rA.y * iA - rB.y * iB;
        K.ex.y = K.ey.x;
        K.ey.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
        K.ez.y = rA.x * iA + rB.x * iB;
        K.ex.z = K.ez.x;
        K.ey.z = K.ez.y;
        K.ez.z = iA + iB;

        if (this.m_frequencyHz > 0.0) {
            var C1 = b2Math.AddVV(cB + rB);
            C1.Subtract(cA); C1.Subtract(rA);

            positionError = C1.Length();
            angularError = 0.0;

            var P = K.Solve22(C1).GetNegative();

            cA.Subtract(b2Math.MulFV(mA, P));
            aA -= iA * b2Math.CrossVV(rA, P);

            cB.Add(b2Math.MulFV(mB, P));
            aB += iB * b2Math.CrossVV(rB, P);
        } else {
            var C1 = b2Math.AddVV(cB, rB);
            C1.Subtract(cA); C1.Subtract(rA);
            var C2 = aB - aA - this.m_referenceAngle;

            positionError = C1.Length();
            angularError = b2Math.Abs(C2);

            var C = new b2Vec3(C1.x, C1.y, C2);

            var impulse = K.Solve33(C).GetNegative();
            var P = new b2Vec2(impulse.x, impulse.y);

            cA.Subtract(b2Math.MulFV(mA, P));
            aA -= iA * (b2Math.CrossVV(rA, P) + impulse.z);

            cB.Add(b2Math.MulFV(mB, P));
            aB += iB * (b2Math.CrossVV(rB, P) + impulse.z);
        }

//        data.positions[this.m_indexA].c.SetV(cA);
        data.positions[this.m_indexA].a = aA;
//        data.positions[this.m_indexB].c.SetV(cB);
        data.positions[this.m_indexB].a = aB;

        return positionError <= b2Settings.b2_linearSlop && angularError <= b2Settings.b2_angularSlop;
    }
    Box2D.inherit(b2WeldJointDef, Box2D.Dynamics.Joints.b2JointDef);
    b2WeldJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
    b2WeldJointDef.b2WeldJointDef = function() {
        Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
        this.localAnchorA = new b2Vec2();
        this.localAnchorB = new b2Vec2();
    };
    b2WeldJointDef.prototype.b2WeldJointDef = function() {
        this.__super.b2JointDef.call(this);
        this.type = b2Joint.e_weldJoint;
        this.localAnchorA = new b2Vec2();
        this.localAnchorB = new b2Vec2();
        this.referenceAngle = 0.0;
    }
    b2WeldJointDef.prototype.Initialize = function(bA, bB, anchor) {
        this.bodyA = bA;
        this.bodyB = bB;
        this.localAnchorA.SetV(this.bodyA.GetLocalPoint(anchor));
        this.localAnchorB.SetV(this.bodyB.GetLocalPoint(anchor));
        this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
    }
    Box2D.inherit(b2WheelJoint, Box2D.Dynamics.Joints.b2Joint);
    b2WheelJoint.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
    b2WheelJoint.b2WheelJoint = function() {
        Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
        this.m_localAnchorA = new b2Vec2();
        this.m_localAnchorB = new b2Vec2();
        this.m_localXAxisA = new b2Vec2();
        this.m_localYAxisA = new b2Vec2();
        this.m_localCenterA = new b2Vec2();
        this.m_localCenterB = new b2Vec2();
        this.m_ax = new b2Vec2();
        this.m_ay = new b2Vec2();
    };
    b2WheelJoint.prototype.GetLocalAnchorA = function() {
        return this.m_localAnchorA;
    }
    b2WheelJoint.prototype.GetLocalAnchorB = function() {
        return this.m_localAnchorB;
    }
    b2WheelJoint.prototype.GetLocalAxisA = function() {
        return this.m_localXAxisA;
    }
    b2WheelJoint.prototype.GetSpringFrequencyHz = function() {
        return this.m_frequencyHz;
    }
    b2WheelJoint.prototype.SetSpringFrequencyHz = function(value) {
        this.m_frequencyHz = value;
    }
    b2WheelJoint.prototype.GetSpringDampingRatio = function() {
        return this.m_dampingRatio;
    }
    b2WheelJoint.prototype.SetSpringDampingRatio = function(value) {
        this.m_dampingRatio = value;
    }
    b2WheelJoint.prototype.GetMotorSpeed = function() {
        return this.m_motorSpeed;
    }
    b2WheelJoint.prototype.SetMotorSpeed = function(speed) {
        this.m_bodyA.SetAwake(true);
        this.m_bodyB.SetAwake(true);
        this.m_motorSpeed = speed;
    }
    b2WheelJoint.prototype.GetMaxMotorTorque = function() {
        return this.m_maxMotorTorque;
    }
    b2WheelJoint.prototype.SetMaxMotorTorque = function(torque) {
        this.m_bodyA.SetAwake(true);
        this.m_bodyB.SetAwake(true);
        this.m_maxMotorTorque = torque;
    }
    b2WheelJoint.prototype.GetMotorTorque = function(inv_dt) {
        return inv_dt * this.m_motorImpulse;
    }
    b2WheelJoint.prototype.GetJointTranslation = function() {
        var bA = this.m_bodyA;
        var bB = this.m_bodyB;
        var pA = bA.GetWorldPoint(this.m_localAnchorA);
        var pB = bB.GetWorldPoint(this.m_localAnchorB);
        var d = b2Math.SubtractVV(pB, pA);
        var axis = bA.GetWorldVector(this.m_localXAxisA);
        var translation = b2Math.DotVV(d, axis);
        return translation;
    }
    b2WheelJoint.prototype.GetJointSpeed = function() {
        var wA = this.m_bodyA.m_angularVelocity;
        var wB = this.m_bodyB.m_angularVelocity;
        return wB - wA;
    }
    b2WheelJoint.prototype.IsMotorEnabled = function() {
        return this.m_enableMotor;
    }
    b2WheelJoint.prototype.EnableMotor = function(flag) {
        this.m_bodyA.SetAwake(true);
        this.m_bodyB.SetAwake(true);
        this.m_enableMotor = flag;
    }
    b2WheelJoint.prototype.GetAnchorA = function() {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
    }
    b2WheelJoint.prototype.GetAnchorB = function() {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
    }
    b2WheelJoint.prototype.GetReactionForce = function(inv_dt) {
        var v1X = this.m_ay.x * this.m_impulse;
        var v1Y = this.m_ay.y * this.m_impulse;
        var v2X = this.m_ax.x * this.m_springImpulse;
        var v2Y = this.m_ax.y * this.m_springImpulse;
        var v = new b2Vec2();
        v.x = v1X + v2X;
        v.y = v1Y + v2Y;
        v.Multiply(inv_dt);
        return v;
    }
    b2WheelJoint.prototype.GetReactionTorque = function(inv_dt) {
        return inv_dt * this.m_motorImpulse;
    }
    b2WheelJoint.prototype.b2WheelJoint = function(def) {
        this.__super.b2Joint.call(this, def);

        this.m_localAnchorA.SetV(def.localAnchorA);
        this.m_localAnchorB.SetV(def.localAnchorB);
        this.m_localXAxisA.SetV(def.localAxisA);
        this.m_localYAxisA = b2Math.CrossFV(1.0, this.m_localXAxisA);

        this.m_mass = 0.0;
        this.m_impulse = 0.0;
        this.m_motorMass = 0.0;
        this.m_motorImpulse = 0.0;
        this.m_springMass = 0.0;
        this.m_springImpulse = 0.0;

        this.m_maxMotorTorque = def.maxMotorTorque;
        this.m_motorSpeed = def.motorSpeed;
        this.m_enableMotor = def.enableMotor;

        this.m_frequencyHz = def.frequencyHz;
        this.m_dampingRatio = def.dampingRatio;

        this.m_bias = 0.0;
        this.m_gamma = 0.0;

        this.m_ax.SetZero();
        this.m_ay.SetZero();
    }
    b2WheelJoint.prototype.InitVelocityConstraints = function(data) {
        this.m_indexA = this.m_bodyA.m_islandIndex;
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_localCenterA.SetV(this.m_bodyA.m_sweep.localCenter);
        this.m_localCenterB.SetV(this.m_bodyB.m_sweep.localCenter);
        this.m_invMassA = this.m_bodyA.m_invMass;
        this.m_invMassB = this.m_bodyB.m_invMass;
        this.m_invIA = this.m_bodyA.m_invI;
        this.m_invIB = this.m_bodyB.m_invI;

        var mA = this.m_invMassA, mB = this.m_invMassB;
        var iA = this.m_invIA, iB = this.m_invIB;

        var cA = data.positions[this.m_indexA].c;
        var aA = data.positions[this.m_indexA].a;
        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;

        var cB = data.positions[this.m_indexB].c;
        var aB = data.positions[this.m_indexB].a;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;

        var qA = new b2Rot(aA), qB = new b2Rot(aB);

        // Compute the effective masses.
        var rA = b2Math.MulRV(qA, b2Math.SubtractVV(this.m_localAnchorA, this.m_localCenterA));
        var rB = b2Math.MulRV(qB, b2Math.SubtractVV(this.m_localAnchorB, this.m_localCenterB));
        var d = b2Math.AddVV(cB, rB);
        d.Subtract(cA); d.Subtract(rA);

        // Point to line constraint
        {
            this.m_ay = b2Math.MulRV(qA, this.m_localYAxisA);
            this.m_sAy = b2Math.CrossVV(b2Math.AddVV(d, rA), this.m_ay);
            this.m_sBy = b2Math.CrossVV(rB, this.m_ay);

            this.m_mass = mA + mB + iA * this.m_sAy * this.m_sAy + iB * this.m_sBy * this.m_sBy;

            if (this.m_mass > 0.0) {
                this.m_mass = 1.0 / this.m_mass;
            }
        }

        // Spring constraint
        this.m_springMass = 0.0;
        this.m_bias = 0.0;
        this.m_gamma = 0.0;
        if (this.m_frequencyHz > 0.0) {
            this.m_ax = b2Math.MulRV(qA, this.m_localXAxisA);
            this.m_sAx = b2Math.CrossVV(b2Math.AddVV(d, rA), this.m_ax);
            this.m_sBx = b2Math.CrossVV(rB, this.m_ax);

            var invMass = mA + mB + iA * this.m_sAx * this.m_sAx + iB * this.m_sBx * this.m_sBx;

            if (invMass > 0.0) {
                this.m_springMass = 1.0 / invMass;

                var C = b2Math.DotVV(d, this.m_ax);

                // Frequency
                var omega = 2.0 * b2Settings.b2_pi * this.m_frequencyHz;

                // Damping coefficient
                var d = 2.0 * this.m_springMass * this.m_dampingRatio * omega;

                // Spring stiffness
                var k = this.m_springMass * omega * omega;

                // magic formulas
                var h = data.step.dt;
                this.m_gamma = h * (d + h * k);
                if (this.m_gamma > 0.0) {
                    this.m_gamma = 1.0 / this.m_gamma;
                }

                this.m_bias = C * h * k * this.m_gamma;

                this.m_springMass = invMass + this.m_gamma;
                if (this.m_springMass > 0.0) {
                    this.m_springMass = 1.0 / this.m_springMass;
                }
            }
        } else {
            this.m_springImpulse = 0.0;
        }

        // Rotational motor
        if (this.m_enableMotor) {
            this.m_motorMass = iA + iB;
            if (this.m_motorMass > 0.0) {
                this.m_motorMass = 1.0 / this.m_motorMass;
            }
        } else {
            this.m_motorMass = 0.0;
            this.m_motorImpulse = 0.0;
        }

        if (data.step.warmStarting) {
            // Account for variable time step.
            this.m_impulse *= data.step.dtRatio;
            this.m_springImpulse *= data.step.dtRatio;
            this.m_motorImpulse *= data.step.dtRatio;

            var P = b2Math.AddVV(b2Math.MulFV(this.m_impulse, this.m_ay), b2Math.MulFV(this.m_springImpulse, this.m_ax));
            var LA = this.m_impulse * this.m_sAy + this.m_springImpulse * this.m_sAx + this.m_motorImpulse;
            var LB = this.m_impulse * this.m_sBy + this.m_springImpulse * this.m_sBx + this.m_motorImpulse;

            vA.Subtract(b2Math.MulFV(this.m_invMassA, P));
            wA -= this.m_invIA * LA;

            vB.Add(b2Math.MulFV(this.m_invMassB, P));
            wB += this.m_invIB * LB;
        } else {
            this.m_impulse = 0.0;
            this.m_springImpulse = 0.0;
            this.m_motorImpulse = 0.0;
        }

//        data.velocities[this.m_indexA].v.SetV(vA);
        data.velocities[this.m_indexA].w = wA;
//        data.velocities[this.m_indexB].v.SetV(vB);
        data.velocities[this.m_indexB].w = wB;
    }
    b2WheelJoint.prototype.SolveVelocityConstraints = function(data) {
        var mA = this.m_invMassA, mB = this.m_invMassB;
        var iA = this.m_invIA, iB = this.m_invIB;

        var vA = data.velocities[this.m_indexA].v;
        var wA = data.velocities[this.m_indexA].w;
        var vB = data.velocities[this.m_indexB].v;
        var wB = data.velocities[this.m_indexB].w;

        // Solve spring constraint
        {
            var Cdot = b2Math.DotVV(this.m_ax, b2Math.SubtractVV(vB, vA)) + this.m_sBx * wB - this.m_sAx * wA;
            var impulse = -this.m_springMass * (Cdot + this.m_bias + this.m_gamma * this.m_springImpulse);
            this.m_springImpulse += impulse;

            var P = b2Math.MulFV(impulse, this.m_ax);
            var LA = impulse * this.m_sAx;
            var LB = impulse * this.m_sBx;

            vA.Subtract(b2Math.MulFV(mA, P));
            wA -= iA * LA;

            vB.Add(b2Math.MulFV(mB, P));
            wB += iB * LB;
        }

        // Solve rotational motor constraint
        {
            var Cdot = wB - wA - this.m_motorSpeed;
            var impulse = -this.m_motorMass * Cdot;

            var oldImpulse = this.m_motorImpulse;
            var maxImpulse = data.step.dt * this.m_maxMotorTorque;
            this.m_motorImpulse = b2Math.Clamp(this.m_motorImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = this.m_motorImpulse - oldImpulse;

            wA -= iA * impulse;
            wB += iB * impulse;
        }

        // Solve point to line constraint
        {
            var Cdot = b2Math.DotVV(this.m_ay, b2Math.SubtractVV(vB, vA)) + this.m_sBy * wB - this.m_sAy * wA;
            var impulse = -this.m_mass * Cdot;
            this.m_impulse += impulse;

            var P = b2Math.MulFV(impulse, this.m_ay);
            var LA = impulse * this.m_sAy;
            var LB = impulse * this.m_sBy;

            vA.Subtract(b2Math.MulFV(mA, P));
            wA -= iA * LA;

            vB.Add(b2Math.MulFV(mB, P));
            wB += iB * LB;
        }

//        data.velocities[this.m_indexA].v.SetV(vA);
        data.velocities[this.m_indexA].w = wA;
//        data.velocities[this.m_indexB].v.SetV(vB);
        data.velocities[this.m_indexB].w = wB;
    }
    b2WheelJoint.prototype.SolvePositionConstraints = function(data) {
        var cA = data.positions[this.m_indexA].c;
        var aA = data.positions[this.m_indexA].a;
        var cB = data.positions[this.m_indexB].c;
        var aB = data.positions[this.m_indexB].a;

        var qA = new b2Rot(aA), qB = new b2Rot(aB);

        var rA = b2Math.MulRV(qA, b2Math.SubtractVV(this.m_localAnchorA, this.m_localCenterA));
        var rB = b2Math.MulRV(qB, b2Math.SubtractVV(this.m_localAnchorB, this.m_localCenterB));
        var d = b2Math.SubtractVV(cB, cA);
        d.Add(rB); d.Subtract(rA);

        var ay = b2Math.MulRV(qA, this.m_localYAxisA);

        var sAy = b2Math.CrossVV(b2Math.AddVV(d, rA), ay);
        var sBy = b2Math.CrossVV(rB, ay);

        var C = b2Math.DotVV(d, ay);

        var k = this.m_invMassA + this.m_invMassB + this.m_invIA * this.m_sAy * this.m_sAy + this.m_invIB * this.m_sBy * this.m_sBy;

        var impulse;
        if (k != 0.0) {
            impulse = - C / k;
        } else {
            impulse = 0.0;
        }

        var P = b2Math.MulFV(impulse, ay);
        var LA = impulse * sAy;
        var LB = impulse * sBy;

        cA.Subtract(b2Math.MulFV(this.m_invMassA, P));
        aA -= this.m_invIA * LA;
        cB.Add(b2Math.MulFV(this.m_invMassB, P));
        aB += this.m_invIB * LB;

//        data.positions[this.m_indexA].c.SetV(cA);
        data.positions[this.m_indexA].a = aA;
//        data.positions[this.m_indexB].c.SetV(cB);
        data.positions[this.m_indexB].a = aB;

        return b2Math.Abs(C) <= b2Settings.b2_linearSlop;
    }
    Box2D.inherit(b2WheelJointDef, Box2D.Dynamics.Joints.b2JointDef);
    b2WheelJointDef.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
    b2WheelJointDef.b2WheelJointDef = function() {
        Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
        this.localAnchorA = new b2Vec2();
        this.localAnchorB = new b2Vec2();
        this.localAxisA = new b2Vec2();
    };
    b2WheelJointDef.prototype.b2WheelJointDef = function() {
        this.__super.b2JointDef.call(this);
        this.type = b2Joint.e_wheelJoint;
        this.localAxisA.Set(1.0, 0.0);
        this.enableMotor = false;
        this.maxMotorTorque = 0.0;
        this.motorSpeed = 0.0;
        this.frequencyHz = 2.0;
        this.dampingRatio = 0.7;
    }
    b2WheelJointDef.prototype.Initialize = function(bA, bB, anchor, axis) {
        this.bodyA = bA;
        this.bodyB = bB;
        this.localAnchorA.SetV(this.bodyA.GetLocalPoint(anchor));
        this.localAnchorB.SetV(this.bodyB.GetLocalPoint(anchor));
        this.localAxisA.SetV(this.bodyA.GetLocalVector(axis));
    }
})();
(function() {
    var b2Draw = Box2D.Dynamics.b2Draw;
    b2Draw.b2Draw = function() {
        this.m_drawScale = 1.0;
        this.m_lineThickness = 1.0;
        this.m_alpha = 1.0;
        this.m_fillAlpha = 1.0;
        this.m_xformScale = 1.0;
        var __this = this;
        //#WORKAROUND
        this.m_sprite = {
            graphics: {
                clear: function() {
                    __this.m_ctx.clearRect(0, 0, __this.m_ctx.canvas.width, __this.m_ctx.canvas.height)
                }
            }
        };
    };
    b2Draw.prototype._color = function(color, alpha) {
        return "rgba(" + ((color & 0xFF0000) >> 16) + "," + ((color & 0xFF00) >> 8) + "," + (color & 0xFF) + "," + alpha + ")";
    };
    b2Draw.prototype.b2Draw = function() {
        this.m_drawFlags = 0;
    };
    b2Draw.prototype.SetFlags = function(flags) {
        if (flags === undefined) flags = 0;
        this.m_drawFlags = flags;
    };
    b2Draw.prototype.GetFlags = function() {
        return this.m_drawFlags;
    };
    b2Draw.prototype.AppendFlags = function(flags) {
        if (flags === undefined) flags = 0;
        this.m_drawFlags |= flags;
    };
    b2Draw.prototype.ClearFlags = function(flags) {
        if (flags === undefined) flags = 0;
        this.m_drawFlags &= ~flags;
    };
    b2Draw.prototype.SetSprite = function(sprite) {
        this.m_ctx = sprite;
    };
    b2Draw.prototype.GetSprite = function() {
        return this.m_ctx;
    };
    b2Draw.prototype.SetDrawScale = function(drawScale) {
        if (drawScale === undefined) drawScale = 0;
        this.m_drawScale = drawScale;
    };
    b2Draw.prototype.GetDrawScale = function() {
        return this.m_drawScale;
    };
    b2Draw.prototype.SetLineThickness = function(lineThickness) {
        if (lineThickness === undefined) lineThickness = 0;
        this.m_lineThickness = lineThickness;
        this.m_ctx.strokeWidth = lineThickness;
    };
    b2Draw.prototype.GetLineThickness = function() {
        return this.m_lineThickness;
    };
    b2Draw.prototype.SetAlpha = function(alpha) {
        if (alpha === undefined) alpha = 0;
        this.m_alpha = alpha;
    };
    b2Draw.prototype.GetAlpha = function() {
        return this.m_alpha;
    };
    b2Draw.prototype.SetFillAlpha = function(alpha) {
        if (alpha === undefined) alpha = 0;
        this.m_fillAlpha = alpha;
    };
    b2Draw.prototype.GetFillAlpha = function() {
        return this.m_fillAlpha;
    };
    b2Draw.prototype.SetXFormScale = function(xformScale) {
        if (xformScale === undefined) xformScale = 0;
        this.m_xformScale = xformScale;
    };
    b2Draw.prototype.GetXFormScale = function() {
        return this.m_xformScale;
    };
    b2Draw.prototype.DrawPolygon = function(vertices, vertexCount, color) {
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
    b2Draw.prototype.DrawSolidPolygon = function(vertices, vertexCount, color) {
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
    b2Draw.prototype.DrawCircle = function(center, radius, color) {
        if (!radius) return;
        var s = this.m_ctx;
        var drawScale = this.m_drawScale;
        s.beginPath();
        s.strokeStyle = this._color(color.color, this.m_alpha);
        s.arc(center.x * drawScale, center.y * drawScale, radius * drawScale, 0, Math.PI * 2, true);
        s.closePath();
        s.stroke();
    };
    b2Draw.prototype.DrawSolidCircle = function(center, radius, axis, color) {
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
    b2Draw.prototype.DrawSegment = function(p1, p2, color) {
        var s = this.m_ctx,
            drawScale = this.m_drawScale;
        s.strokeStyle = this._color(color.color, this.m_alpha);
        s.beginPath();
        s.moveTo(p1.x * drawScale, p1.y * drawScale);
        s.lineTo(p2.x * drawScale, p2.y * drawScale);
        s.closePath();
        s.stroke();
    };
    b2Draw.prototype.DrawTransform = function(xf) {
        var s = this.m_ctx,
            drawScale = this.m_drawScale;
        s.beginPath();
        s.strokeStyle = this._color(0xff0000, this.m_alpha);
        s.moveTo(xf.p.x * drawScale, xf.p.y * drawScale);
        s.lineTo((xf.p.x + this.m_xformScale * xf.q.c) * drawScale, (xf.p.y + this.m_xformScale * xf.q.s) * drawScale);

        s.strokeStyle = this._color(0xff00, this.m_alpha);
        s.moveTo(xf.p.x * drawScale, xf.p.y * drawScale);
        s.lineTo((xf.p.x - this.m_xformScale * xf.q.s) * drawScale, (xf.p.y + this.m_xformScale * xf.q.c) * drawScale);
        s.closePath();
        s.stroke();
    };
})(); //post-definitions
var i;
for (i = 0; i < Box2D.postDefs.length; ++i) Box2D.postDefs[i]();
delete Box2D.postDefs;

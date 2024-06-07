import * as Physics from './physicsEngine.js';

let Vec2 = Physics.Vec2;
let Rigidbody = Physics.Rigidbody;
let PhysicsEngine = Physics.PhysicsEngine;
let RectangleCollider = Physics.RectangleCollider;
let CircleCollider = Physics.CircleCollider;
let TriangleCollider = Physics.TriangleCollider;

const fullEngineDemo = function(p) {
    let engine = new Physics.PhysicsEngine(50)
    // setup test senario
    const rigidBody1 = new Rigidbody(new Vec2(100, 100), 0, 1, 1, []);
    const rigidBody2 = new Rigidbody(new Vec2(300, -100), 0, 1, 1, []);
    const rigidBody3 = new Rigidbody(new Vec2(200, 300), 0, 1, 1, []);


    rigidBody1.addCollider(new RectangleCollider(rigidBody1, 0, 0, 35, 1, 100, 100));
    rigidBody1.addCollider(new CircleCollider(rigidBody1, 50, 0, 1, 20));

   

    const ground = new Rigidbody(new Vec2(300, 400), Math.PI/7, Infinity, 1, []);
    ground.addCollider(new RectangleCollider(ground, 0, 0, 0, 1, 500, 100));

    engine.addRigidbody(rigidBody1);
    engine.addRigidbody(ground);


  p.setup = function() {
    const mainDivWidth = document.getElementsByTagName('main')[0].getBoundingClientRect().width;
    const canvas = p.createCanvas(mainDivWidth, 600);
    canvas.parent('full-engine-holder');


  }

  p.draw = function() {
    p.background(220);
    engine.stepSimulation(p.deltaTime/1000);
    
    p.fill(0);  
    for (let i = 0; i < engine.rigidBodies.length; i++) {
      for (let j = 0; j < engine.rigidBodies[i].colliders.length; j++) {
        let collider = engine.rigidBodies[i].colliders[j];
        if (collider instanceof Physics.CircleCollider) {
          p.ellipse(collider.position.x, collider.position.y, collider.radius * 2);
        }

        if (collider instanceof Physics.ConvexCollider) {
            let ctx = p.drawingContext;

            ctx.beginPath();
            ctx.strokeStyle = 'grey';
            ctx.moveTo(collider.vertices[0].x, collider.vertices[0].y);
            for (let i = 1; i < collider.vertices.length; i++){
                ctx.lineTo(collider.vertices[i].x, collider.vertices[i].y);
            }
            ctx.lineTo(collider.vertices[0].x, collider.vertices[0].y);
            ctx.stroke();
            ctx.closePath();
        }
      }
    }
   

  }

  p.windowResized = function() {
    const mainDivWidth = document.getElementsByTagName('main')[0].getBoundingClientRect().width;
    p.resizeCanvas(mainDivWidth, 600);
  }

}



let fullEngineSketch = new p5(fullEngineDemo);

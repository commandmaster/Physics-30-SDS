import * as Physics from './physicsEngine.js';

let Vec2 = Physics.Vec2;
let Rigidbody = Physics.Rigidbody;
let PhysicsEngine = Physics.PhysicsEngine;

const fullEngineDemo = function(p) {
    let engine = new Physics.PhysicsEngine(50)
    let rb1 = new Rigidbody(new Vec2(100, 100), 0, 1, 1, []);
    let collider = new Physics.CircleCollider(rb1, 0, 0, 1, 10);
    rb1.addCollider(collider);

    let rb2 = new Rigidbody(new Vec2(200, 200), 0, 1, 1, []);
    let collider2 = new Physics.CircleCollider(rb2, 0, 0, 1, 10);
    rb2.addCollider(collider2);

    let floor = new Rigidbody(new Vec2(0, 500), 0, 1, 0, []);
    let floorCollider = new Physics.RectangleCollider(floor, 0, 200, 0, 1, 500, 100);
    
    floor.isStatic = true;
    floor.addCollider(floorCollider);

    engine.addRigidbody(rb1);
    engine.addRigidbody(rb2);
    engine.addRigidbody(floor);

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
            // ctx.beginPath();
            // ctx.strokeStyle = 'grey';
            // ctx.moveTo(collider.vertices[0].x, collider.vertices[0].y);
            // for (let i = 1; i < collider.vertices.length; i++){
            //     ctx.lineTo(collider.vertices[i].x, collider.vertices[i].y);
            // }
            // ctx.lineTo(collider.vertices[0].x, collider.vertices[0].y);
            // ctx.stroke();
            // ctx.closePath();

            p.beginShape();
            p.stroke(0);
            p.strokeWeight(1);
            p.fill(200);
            for (let i = 0; i < collider.vertices.length; i++) {
              p.vertex(collider.vertices[i].x, collider.vertices[i].y);
            }
            p.endShape(p.CLOSE);
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

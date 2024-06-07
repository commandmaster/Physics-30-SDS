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



    rigidBody1.addCollider(new RectangleCollider(rigidBody1, 0, 0, 35, 1, 100, 100));

    window.addEventListener('mousedown', (e) => {
      e.preventDefault();
      const worldPos = new Vec2(p.mouseX, p.mouseY); 
  
      if (e.button === 1){
           for (let i = 0; i < 100; i++){
                const newRB = new Rigidbody(new Vec2(worldPos.x + i * 5, worldPos.y + Math.random()*150 - 75), 0, 1, 1, []);
                newRB.addCollider(new CircleCollider(newRB, 0, 0, 1, 10));
                engine.addRigidbody(newRB); 
            }
      }
    });

    const ground = new Rigidbody(new Vec2(300, 400), Math.PI/20, Infinity, 1, []);
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
          let ctx = p.drawingContext;
          ctx.beginPath();
          ctx.arc(collider.position.x, collider.position.y, collider.radius, 0, 2 * Math.PI);
          ctx.stroke();
          ctx.closePath();


            // ctx.beginPath();
            // ctx.strokeStyle = 'black';
            
            // const boundingBox = collider.boundingBox;
            // const topLeft = boundingBox.position;
            // const bottomRight = new Vec2(boundingBox.position.x + boundingBox.width, boundingBox.position.y + boundingBox.height);

            // ctx.moveTo(topLeft.x, topLeft.y);
            // ctx.lineTo(bottomRight.x, topLeft.y);
            // ctx.lineTo(bottomRight.x, bottomRight.y);
            // ctx.lineTo(topLeft.x, bottomRight.y);
            // ctx.lineTo(topLeft.x, topLeft.y);
            // ctx.stroke();
            // ctx.closePath();
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

            // draw the bounding box
            // ctx.beginPath();
            // ctx.strokeStyle = 'black';
            
            // const boundingBox = collider.boundingBox;
            // const topLeft = boundingBox.position;
            // const bottomRight = new Vec2(boundingBox.position.x + boundingBox.width, boundingBox.position.y + boundingBox.height);

            // ctx.moveTo(topLeft.x, topLeft.y);
            // ctx.lineTo(bottomRight.x, topLeft.y);
            // ctx.lineTo(bottomRight.x, bottomRight.y);
            // ctx.lineTo(topLeft.x, bottomRight.y);
            // ctx.lineTo(topLeft.x, topLeft.y);
            // ctx.stroke();
            // ctx.closePath();

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

import * as Physics from './physicsEngine.js';

let Vec2 = Physics.Vec2;
let Rigidbody = Physics.Rigidbody;
let PhysicsEngine = Physics.PhysicsEngine;
let RectangleCollider = Physics.RectangleCollider;
let CircleCollider = Physics.CircleCollider;
let TriangleCollider = Physics.TriangleCollider;

const poolDemo = function(p) {
    let engine = new Physics.PhysicsEngine(50)
    let queueBall = new Rigidbody(new Vec2(-150, 0), 0, 1, 1, []);
    queueBall.addCollider(new CircleCollider(queueBall, 0, 0, 1, 15));
    queueBall.acceleration = new Vec2(0, 0);

    engine.addRigidbody(queueBall);

    let queueBallImg;
    let poolTableImg;

    let poolBalls = spawnPoolBalls();

    // Create walls 
    const rightWall = new Rigidbody(new Vec2(530, -130 + 125), 0, Infinity, 1, []);
    rightWall.addCollider(new Physics.ConvexCollider(rightWall, 0, 0, 1, 1, [new Vec2(-250, -200), new Vec2(250, -200), new Vec2(250, 200), new Vec2(-250, 200)]));

    const leftWall = new Rigidbody(new Vec2(-530, -130 + 125), 0, Infinity, 1, []);
    leftWall.addCollider(new Physics.ConvexCollider(leftWall, 0, 0, 1, 1, [new Vec2(-250, -200), new Vec2(250, -200), new Vec2(250, 200), new Vec2(-250, 200)]));

    const topWall = new Rigidbody(new Vec2(0, -468 + 125), 0, Infinity, 1, []);
    topWall.addCollider(new Physics.ConvexCollider(topWall, 0, 0, 1, 1, [new Vec2(-280, -200), new Vec2(280, -200), new Vec2(280, 200), new Vec2(-280, 200)]));

    const bottomWall = new Rigidbody(new Vec2(0, 468 - 125), 0, Infinity, 1, []);
    bottomWall.addCollider(new Physics.ConvexCollider(bottomWall, 0, 0, 1, 1, [new Vec2(-280, -200), new Vec2(280, -200), new Vec2(280, 200), new Vec2(-280, 200)])); 
    
    
    engine.addRigidbody(rightWall);
    engine.addRigidbody(leftWall);
    engine.addRigidbody(topWall);
    engine.addRigidbody(bottomWall);

    p.preload = function() {
        queueBallImg = p.loadImage('images/queueBall.png');
        poolTableImg = p.loadImage('images/poolTable.png');
    }

    p.setup = function() {
        const mainDivWidth = document.getElementsByTagName('main')[0].getBoundingClientRect().width;
        const canvas = p.createCanvas(mainDivWidth, 600);
        canvas.parent('pool-demo-holder');

        p.imageMode(p.CENTER);
        p.noStroke();

        window.addEventListener('keydown', (e) => {
            e.preventDefault();
            const worldPos = new Vec2(p.mouseX - p.width/2, p.mouseY - p.height/2); 
        
            if (e.key === ' ') {
                queueBall.applyImpulse(Vec2.sub(queueBall.position, worldPos ).normalize().scale(700), 1); 
            }
        });
    }

    p.draw = function() {
        p.background(17,144,41);
        engine.stepSimulation(p.deltaTime/1000);

        const worldPos = new Vec2(p.mouseX - p.width/2, p.mouseY - p.height/2);
        const deleteQueue = new Set();

        // check if pool ball is in pocket
        for (let i = 0; i < poolBalls.length; i++) {
            if (poolBalls[i].position.x > 259 && poolBalls[i].position.y < -125 ) {
                deleteQueue.add(poolBalls[i]);
            }

            if (poolBalls[i].position.x > 259 && poolBalls[i].position.y > 120) {
                deleteQueue.add(poolBalls[i]);
            } 

            if (poolBalls[i].position.x < -259 && poolBalls[i].position.y < -125) {
                deleteQueue.add(poolBalls[i]);
            }

            if (poolBalls[i].position.x < -259 && poolBalls[i].position.y > 120) {
                deleteQueue.add(poolBalls[i]);
            } 

            if (poolBalls[i].position.x > -20 && poolBalls[i].position.x < 20 && poolBalls[i].position.y < -125) {
                deleteQueue.add(poolBalls[i]);
            }

            if (poolBalls[i].position.x > -20 && poolBalls[i].position.x < 20 && poolBalls[i].position.y > 120) {
                deleteQueue.add(poolBalls[i]);
            }
        }

        for (let ball of deleteQueue) {
            poolBalls.splice(poolBalls.indexOf(ball), 1);
            engine.deleteRigidbody(ball);
        }

        if (poolBalls.length <= 0) {
            poolBalls = spawnPoolBalls();
        }

        p.translate(p.width/2, p.height/2)

        if (poolTableImg) p.image(poolTableImg, 0, 0, 1280/2, 731/2);

        p.stroke(255)
        p.strokeWeight(2);
        p.circle(queueBall.position.x, queueBall.position.y, 30);
        if (queueBallImg) p.image(queueBallImg, queueBall.position.x, queueBall.position.y, 30, 30);
        p.noStroke();

        for (let i = 0; i < poolBalls.length; i++) {
            p.fill(poolBalls[i].color.r, poolBalls[i].color.g, poolBalls[i].color.b);
            p.ellipse(poolBalls[i].position.x, poolBalls[i].position.y, poolBalls[i].colliders[0].radius*2);
        }

        // debug the walls 
        p.fill(0);
        // for (let i = 0; i < walls.colliders.length; i++) {
        //     let collider = walls.colliders[i];
        //     console.log(collider);
        //     if (collider instanceof Physics.ConvexCollider) {
        //         const vertices = collider.vertices;
        //         p.beginShape();
        //         for (let j = 0; j < vertices.length; j++) {
        //             p.vertex(vertices[j].x, vertices[j].y);
        //         }
        //         p.endShape(p.CLOSE);

        //     } else if (collider instanceof CircleCollider) {
        //         p.ellipse(collider.position.x, collider.position.y, collider.radius*2);
        //     }
        // }

        //  
    }

    p.windowResized = function() {
        const mainDivWidth = document.getElementsByTagName('main')[0].getBoundingClientRect().width;
        p.resizeCanvas(mainDivWidth, 600);
    }

    function spawnPoolBalls() {
        const ballsToDraw = [];

            let positions = [
                new Vec2(0, 0),
                new Vec2(30, -15), new Vec2(30, 15),
                new Vec2(60, -30), new Vec2(60, 0), new Vec2(60, 30),
                new Vec2(90, -45), new Vec2(90, -15), new Vec2(90, 15), new Vec2(90, 45),
                new Vec2(120, -60), new Vec2(120, -30), new Vec2(120, 0), new Vec2(120, 30), new Vec2(120, 60)
            ]; 
        for (let i = 0; i < 15; i++) {
            // Spawn balls in a triangle formation
            let position = positions[i];

            const newRB = new Rigidbody(position, 0, 1, 1, []);
            
            newRB.acceleration = new Vec2(0, 0);
            newRB.addCollider(new CircleCollider(newRB, 0, 0, 1, 15));
            engine.addRigidbody(newRB);  

            newRB.color = {r: Math.random() * 255, g: Math.random() * 255, b: Math.random() * 255};

            ballsToDraw.push(newRB);
        }

        return ballsToDraw;
    }

}



let poolDemoSketch = new p5(poolDemo);

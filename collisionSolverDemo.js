
const collisionSolverDemo = function(p) {
  let simulation;


  p.setup = function() {
    const mainDivWidth = document.getElementsByTagName('main')[0].getBoundingClientRect().width;
    const canvas = p.createCanvas(mainDivWidth, 600);
    canvas.parent('collision-solver-holder');

    simulation = new NewSimulation(p, 300);
    
    const rb1 = new NewRigidbody(p, 1, p.createVector(100, 100), p.createVector(0, 0), p.createVector(0, 0), 10);
    const rb2 = new NewRigidbody(p, 1, p.createVector(200, 100), p.createVector(0, 0), p.createVector(0, 0), 10);

    simulation.addRigidbody(rb1);
    simulation.addRigidbody(rb2);

    rb1.applyImpulse(p.createVector(400, 0), 1);
    rb2.applyImpulse(p.createVector(-400, 0), 1);
    
    const addRigidbodyButton = p.createButton('Add Rigid Body');
    addRigidbodyButton.parent('collision-solver-holder');
    addRigidbodyButton.size(150, 30);
    addRigidbodyButton.style('padding', '0');
    addRigidbodyButton.style('color', 'black');
    addRigidbodyButton.style('background-color', 'rgb(220, 220, 220)');
    addRigidbodyButton.style('border', 'none');

    addRigidbodyButton.mousePressed(() => {
        const rb = new NewRigidbody(p, 1, p.createVector(p.random(0, p.width), p.random(0, p.height)), p.createVector(p.random(-100, 100), p.random(-100, 100)), p.createVector(0, 0), 10);
        rb.applyImpulse(p.createVector(p.random(-400, 400), p.random(-100, 100)), 2);
        simulation.addRigidbody(rb);
    });

    p.noStroke();
  }

  p.draw = function() {
    p.background(220);

    simulation.stepSimulation(p.deltaTime/1000);
    simulation.draw(p);
  }

  p.windowResized = function() {
    const mainDivWidth = document.getElementsByTagName('main')[0].getBoundingClientRect().width;
    p.resizeCanvas(mainDivWidth, 600);
  }

}

class NewSimulation{
  constructor(p, gravityStrength = 9.8) {
    this.p = p;
    this.rigidbodies = [];
    this.gravityStrength = gravityStrength;

    this.collisionSolver = new CollisionSolver(this);
    this.totalSimulationMomentum = 0;
    this.totalSimulationEnergy = 0;
  }

  addRigidbody(rigidbody) {
    this.rigidbodies.push(rigidbody);
  }

  stepSimulation(dt) {
    this.collisionSolver.resolveCollisions();
    this.totalSimulationMomentum = 0;
    this.totalSimulationEnergy = 0;

    for(let i = 0; i < this.rigidbodies.length; i++) {
        let rb = this.rigidbodies[i];
        rb.applyForce(this.p.createVector(0, this.gravityStrength * rb.mass));
        rb.stepSimulation(dt);

        if (rb.position.x < 0 + rb.radius || rb.position.x > this.p.width - rb.radius) {
            rb.position.x = this.p.constrain(rb.position.x, rb.radius, this.p.width - rb.radius);
            rb.velocity.x *= -1;
        }

        if (rb.position.y < 0 + rb.radius || rb.position.y > this.p.height - rb.radius) {
            rb.position.y = this.p.constrain(rb.position.y, rb.radius, this.p.height - rb.radius);
            rb.velocity.y *= -1;
        }

        this.totalSimulationEnergy += (0.5 * rb.mass * rb.velocity.magSq()) + (this.gravityStrength * rb.mass * -(rb.position.y - this.p.height));
        this.totalSimulationMomentum += rb.velocity.mag() * rb.mass;
    } 
  }

  draw(p) {
    p.fill(0);
    p.textSize(20);
    p.text(`Total Momentum: ${this.totalSimulationMomentum.toFixed(2)} Wu (Ns)`, 10, 30);
    p.text(`Total Energy: ${this.totalSimulationEnergy.toFixed(2)} J`, 10, 60);

    for(let i = 0; i < this.rigidbodies.length; i++) {
        let rb = this.rigidbodies[i];
        p.fill(rb.color);
        p.ellipse(rb.position.x, rb.position.y, rb.radius*2);
    }
  } 
}

class NewRigidbody {
  constructor(p, mass, position, velocity, acceleration, radius) {
    this.p = p;
    this.mass = mass;
    this.position = position;
    this.velocity = velocity;
    this.acceleration = acceleration;
    this.radius = radius;

    this.color = p.color(255, 0, p.random(255));
  }

  stepSimulation(dt) {
    dt = Math.min(dt, 1);

    this.velocity.add(p5.Vector.mult(this.acceleration, dt));
    this.position.add(p5.Vector.mult(this.velocity, dt));


    this.acceleration.mult(0);
  }

  applyForce(force) {
    this.acceleration.add(p5.Vector.div(force, this.mass));
  }

  applyImpulse(forceVec, time) {
    // use the impulse = F * dt formula
    let impulse = p5.Vector.mult(forceVec, time);
    this.velocity.add(p5.Vector.div(impulse, this.mass));
  }
}

class CollisionSolver{
    constructor(simulation) {
        this.simulation = simulation;
    }

    resolveCollisions() {
        // Resolve all collisions (circle to circle)

        for (let i = 0; i < this.simulation.rigidbodies.length; i++) {
            for (let j = i + 1; j < this.simulation.rigidbodies.length; j++) {
                let rb1 = this.simulation.rigidbodies[i];
                let rb2 = this.simulation.rigidbodies[j];

                let distance = p5.Vector.sub(rb1.position, rb2.position).mag();

                if (distance < rb1.radius + rb2.radius) {
                    // They are colliding
                    let normal = p5.Vector.sub(rb1.position, rb2.position).normalize(); // Normal vector pointing from rb2 to rb1
                    let relativeVelocity = p5.Vector.sub(rb1.velocity, rb2.velocity); // Relative velocity of rb1 to rb2

                    const collisionOfRestitution = 1; // Perfectly elastic collision
                    let impulse = p5.Vector.mult(normal, p5.Vector.dot(normal, relativeVelocity) * (1 + collisionOfRestitution) / (1 / rb1.mass + 1 / rb2.mass));

                    rb1.velocity.sub(p5.Vector.div(impulse, rb1.mass)); // Subtract impulse from velocity of the first object
                    rb2.velocity.add(p5.Vector.div(impulse, rb2.mass)); // Add impulse to velocity of the second object

                    // Move the objects out of each other
                    let overlap = (rb1.radius + rb2.radius) - distance;
                    let correction = p5.Vector.mult(normal, overlap / (1 / rb1.mass + 1 / rb2.mass));

                    rb1.position.add(p5.Vector.div(correction, rb1.mass));
                    rb2.position.sub(p5.Vector.div(correction, rb2.mass));
                }
            }
        }
    }
}

let collisionSolverSketch = new p5(collisionSolverDemo);
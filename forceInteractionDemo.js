let speedSlider;
const forceInteractionDemo = function(p) {
  let simulation;
  let rb1;

  let gravitySlider;
  

  p.setup = function() {
    const mainDivWidth = document.getElementsByTagName('main')[0].getBoundingClientRect().width;
    const canvas = p.createCanvas(mainDivWidth, 600);
    canvas.parent('force-interaction-holder');

    simulation = new Simulation(p, 50);
    rb1 = new Rigidbody(p, 1, p.createVector(100, 100), p.createVector(0, 0), p.createVector(0, 0), 10);
    simulation.addRigidbody(rb1);

    const gravityLabel = p.createP('Gravity: ');
    gravityLabel.parent('force-interaction-holder');

    const speedLabel = p.createP('Applied Force Strength: ');
    speedLabel.parent('force-interaction-holder');


    gravitySlider = p.createSlider(0, 100, 50, 1);
    gravitySlider.parent(gravityLabel);

    speedSlider = p.createSlider(0, 300, 50, 1);
    speedSlider.parent(speedLabel);

    gravitySlider.input(() => {
      simulation.gravityStrength = gravitySlider.value();
    });



    p.noStroke();
  }

  p.draw = function() {
    p.background(220);

    if (p.keyIsDown(p.UP_ARROW) || p.keyIsDown(87)) {
      rb1.applyForce(p.createVector(0, -speedSlider.value()));
    }

    if (p.keyIsDown(p.DOWN_ARROW) || p.keyIsDown(83)) {
      rb1.applyForce(p.createVector(0, speedSlider.value()));
    }

    if (p.keyIsDown(p.LEFT_ARROW) || p.keyIsDown(65)) {
      rb1.applyForce(p.createVector(-speedSlider.value(), 0));
    }

    if (p.keyIsDown(p.RIGHT_ARROW) || p.keyIsDown(68)) {
      rb1.applyForce(p.createVector(speedSlider.value(), 0));
    }

    simulation.stepSimulation(p.deltaTime/1000);
    simulation.draw(p);

  }

  p.windowResized = function() {
    const mainDivWidth = document.getElementsByTagName('main')[0].getBoundingClientRect().width;
    p.resizeCanvas(mainDivWidth, 600);
  }

}

class Simulation{
  constructor(p, gravityStrength = 9.8) {
    this.p = p;
    this.rigidbodies = [];
    this.gravityStrength = gravityStrength;
  }

  addRigidbody(rigidbody) {
    this.rigidbodies.push(rigidbody);
  }

  stepSimulation(dt) {
    for(let i = 0; i < this.rigidbodies.length; i++) {
      let rb = this.rigidbodies[i];
      rb.applyForce(this.p.createVector(0, this.gravityStrength * rb.mass));
      rb.stepSimulation(dt);
    } 
  }

  draw(p) {
    p.textSize(20);
    p.fill(0);
    p.text(`Gravity Strength: ${this.gravityStrength} m*s^-2`, 5, 20)


    p.text(`Applied Force Strength: ${speedSlider.value()} N`, 5, 60)

    p.textSize(15);
    p.text(`Use WASD or Arrow Keys to apply force to the object`, 5, p.height - 20)
    p.text(`1kg object`, 5, p.height - 40)

    for(let i = 0; i < this.rigidbodies.length; i++) {
      let rb = this.rigidbodies[i];
      p.fill(255, 0, 0)
      p.ellipse(rb.position.x, rb.position.y, rb.radius*2);
    }
  } 
}

class Rigidbody {
  constructor(p, mass, position, velocity, acceleration, radius) {
    this.p = p;
    this.mass = mass;
    this.position = position;
    this.velocity = velocity;
    this.acceleration = acceleration;
    this.radius = radius;

  }

  stepSimulation(dt) {
    dt = Math.min(dt, 1);

    this.velocity.add(p5.Vector.mult(this.acceleration, dt));
    this.position.add(p5.Vector.mult(this.velocity, dt));

    if (this.position.y + this.radius > 600) {
      this.position.y = 600 - this.radius;
      this.velocity.y *= -1;
    }

    if (this.position.y - this.radius < 0) {
      this.position.y = this.radius;
      this.velocity.y *= -1;
    }

    if (this.position.x + this.radius > this.p.width) {
      this.position.x = this.p.width - this.radius;
      this.velocity.x *= -1;
    }

    if (this.position.x - this.radius < 0) {
      this.position.x = this.radius;
      this.velocity.x *= -1;
    }

    this.acceleration.mult(0);
  }

  applyForce(force) {
    this.acceleration.add(p5.Vector.div(force, this.mass));
  }
}

let forceInteractionSketch = new p5(forceInteractionDemo);


const demo = function(p) {

  let accSlider;
  let velSlider;
  let framRateSlider;

  let pos;
  let velocity;
  let acceleration;
  let staringRunTime = 3;
  let runTime = staringRunTime;

  p.setup = function() {
    // create the canvas attached to the div with id 'sketch-holder'
    let canvas = p.createCanvas(window.innerWidth, 200 );
    canvas.parent('sketch-holder');

    // Show the values of the sliders
    const accLabel = p.createP('Acceleration: ');
    accLabel.parent('sketch-holder');

    const velLabel = p.createP('Velocity: ');
    velLabel.parent('sketch-holder');

    const frameRateLabel = p.createP('Frame Rate: ');
    frameRateLabel.parent('sketch-holder');

    // create sliders
    accSlider = p.createSlider(0, 50, 10, 1);
    accSlider.parent(accLabel);

    velSlider = p.createSlider(0, 50, 20, 1);
    velSlider.parent(velLabel);

    framRateSlider = p.createSlider(1, 100, 60, 1);
    framRateSlider.parent(frameRateLabel);


    accSlider.input(() => {
      runTime = 0;
    });

    velSlider.input(() => {
      runTime = 0;
    });
    

    p.noStroke();
    p.fill(255, 0, 0)

    pos = p.createVector(0, 0);
    velocity = p.createVector(20, 0);
    acceleration = p.createVector(10, 0);
  }

  p.draw = function() {
    // scale based on the window width
    p.frameRate(framRateSlider.value());
    p.background(220);

    p.push();
    p.translate(20, p.height / 2);

    
    p.scale(window.innerWidth / 800, window.innerWidth / 800);
    // live implict euler integration example

    if (runTime <= 0.001) {
      runTime = staringRunTime;
      pos = p.createVector(0, 0);
      velocity = p.createVector(velSlider.value(), 0);
      acceleration = p.createVector(accSlider.value(), 0);

      return;
    }
    
    const dt = p.deltaTime / 1000;
    runTime -= dt;

    // update velocity
    velocity.add(p5.Vector.mult(acceleration, dt));

    // update position
    pos.add(p5.Vector.mult(velocity, dt));

    // draw position
    p.ellipse(pos.x, pos.y, 25, 25);

    p.fill(0);
    p.ellipse(pos.x, pos.y, 1, 1);

    // Draw the line caluclated by the kinematic equations
    p.stroke(0, 0, 255);
    p.strokeWeight(1);
    
    // Draw the line where the object should finnish

    const startingVelocity = p.createVector(velSlider.value(), 0);
    const startingAcceleration = p.createVector(accSlider.value(), 0);


    const displacement = p5.Vector.mult(startingVelocity, staringRunTime).add(p5.Vector.mult(p5.Vector.mult(startingAcceleration, 0.5), staringRunTime * staringRunTime));
    p.line(displacement.x, 0, displacement.x, p.height);  

    p.textSize(10);

    p.noStroke();
    p.text(`Time left: ${runTime.toFixed(3)}s, Current Position: ${pos.x.toFixed(4)}m`, 0, -30);

    p.fill(0, 0, 255);
    p.text(`Kinematic Prediction: ${displacement.x.toFixed(4)}m`, 0, -20);

    p.pop();


  }

  p.windowResized = function() {
    p.resizeCanvas(window.innerWidth, 200);
  }
}

let p5Instance = new p5(demo);

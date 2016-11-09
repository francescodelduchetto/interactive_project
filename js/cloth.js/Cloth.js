/*
 * Cloth Simulation using a relaxed constraints solver
 */

// Suggested Readings

// Advanced Character Physics by Thomas Jakobsen Character
// http://freespace.virgin.net/hugo.elias/models/m_cloth.htm
// http://en.wikipedia.org/wiki/Cloth_modeling
// http://cg.alexandra.dk/tag/spring-mass-system/
// Real-time Cloth Animation http://www.darwin3d.com/gamedev/articles/col0599.pdf

var DAMPING = 0.03;
var DRAG = 1 - DAMPING;
var MASS = 0.1;
var restDistance = 0.1;

var GRAVITY = 981 * 1.4;
var gravity = new THREE.Vector3( 0, - GRAVITY, 0 ).multiplyScalar( MASS );

var TIMESTEP = 18 / 1000;
var TIMESTEP_SQ = TIMESTEP * TIMESTEP;

function Cloth(width, height, depth, centerPosition) {
	this.geometry;
	this.ball;
	this.width = width;
	this.height = height;
	this.depth = depth;
	this.centerPosition = centerPosition;
	this.pins = [];
	this.tmpForce = new THREE.Vector3();
	this.lastTime;
	this.diff = new THREE.Vector3();

	this.collided_before = [];
	this.some_collided_before = false;
	this.first_contact_vel = new THREE.Vector3(0,0,0);
	this.first_contact_sign = undefined;
	this.first_contact_point = [];

	this.wind = true;
	this.windStrength = 100;
	this.windForce = new THREE.Vector3( 0, 0, 0 );


	w = segs;// || 10;
	h = segs;// || 10;
	this.w = w;
	this.h = h;

	this.particles = [];
	this.constraints = [];
	this.u, this.v;
	var clothFunction = plane( restDistance * segs, restDistance * segs );

	this.setGeometry = function(geometry) {
		this.geometry = geometry;
	}

	this.setBall = function(ball) {
		this.ball = ball;
	}
	// Create particles
	for ( v = 0; v <= this.h; v ++ ) {
		for ( u = 0; u <= this.w; u ++ ) {
			this.particles.push(
				new Particle( u / this.w, v / this.h, MASS )
			);
			this.collided_before.push(false);
		}
	}

	// Structural
	for ( v = 0; v < this.h; v ++ ) {
		for ( u = 0; u < this.w; u ++ ) {
			this.constraints.push( [
				this.particles[ index( u, v ) ],
				this.particles[ index( u, v + 1 ) ],
				restDistance
			] );
			this.constraints.push( [
				this.particles[ index( u, v ) ],
				this.particles[ index( u + 1, v ) ],
				restDistance
			] );
		}
	}
	for ( u = this.w, v = 0; v < this.h; v ++ ) {
		this.constraints.push( [
			this.particles[ index( u, v ) ],
			this.particles[ index( u, v + 1 ) ],
			restDistance
		] );
	}
	for ( v = this.h, u = 0; u < this.w; u ++ ) {
		this.constraints.push( [
			this.particles[ index( u, v ) ],
			this.particles[ index( u + 1, v ) ],
			restDistance
		] );
	}


	function index( u, v ) {
		return u + v * ( this.w + 1 );
	};


	function plane() {
		return function(u,v) {
			var x = 0, y = 0, z = 0;
			if (width != 0) {
				x = centerPosition.x + ( u - 0.5 ) * width;
				y = centerPosition.y + ( v - 0.5) * height;
			} else {
				x = centerPosition.x;
				y = centerPosition.y + ( u - 0.5 ) * height;
			}
			z = centerPosition.z + ( v - 0.5) * depth;
			return new THREE.Vector3( x, y, z );
		}
	};

	function Particle( x, y, mass ) {
		this.position = clothFunction( x, y ); // position
		this.previous = clothFunction( x, y ); // previous
		this.original = clothFunction( x, y );
		this.a = new THREE.Vector3( 0, 0, 0 ); // acceleration
		this.mass = mass;
		this.invMass = 1 / mass;
		this.tmp = new THREE.Vector3();
		this.tmp2 = new THREE.Vector3();
	};

	// Force -> Acceleration
	Particle.prototype.addForce = function( force ) {
		this.a.add(
			this.tmp2.copy( force ).multiplyScalar( this.invMass )
		);
	};

	// Performs Verlet integration
	Particle.prototype.integrate = function( timesq ) {
		var newPos = this.tmp.subVectors( this.position, this.previous );
		newPos.multiplyScalar( DRAG ).add( this.position );
		newPos.add( this.a.multiplyScalar( timesq ) );
		this.tmp = this.previous;
		this.previous = this.position;
		this.position = newPos;
		this.a.set( 0, 0, 0 );
	};


	this.satisifyConstraints = function( p1, p2, distance ) {
		this.diff.subVectors( p2.position, p1.position );
		var currentDist = this.diff.length();
		if ( currentDist === 0 ) return; // prevents division by 0
		var correction = this.diff.multiplyScalar( 1 - distance / currentDist );
		var correctionHalf = correction.multiplyScalar( 0.5 );
		p1.position.add( correctionHalf );
		p2.position.sub( correctionHalf );
	}

	this.simulate = function( time ) {
		if ( ! this.lastTime ) {
			this.lastTime = time;
			return;
		}

		var i, il, particles, particle, pt, constraints, constraint;
		// Aerodynamics forces
		if ( this.wind ) {
			var face, faces = this.geometry.faces, normal;
			particles = this.particles;
			for ( i = 0, il = faces.length; i < il; i ++ ) {
				face = faces[ i ];
				normal = face.normal;
				this.tmpForce.copy( normal ).normalize().multiplyScalar( normal.dot( this.windForce ) );
				particles[ face.a ].addForce( this.tmpForce );
				particles[ face.b ].addForce( this.tmpForce );
				particles[ face.c ].addForce( this.tmpForce );
			}
		}

		for ( particles = this.particles, i = 0, il = particles.length; i < il; i ++ ) {
			particle = particles[ i ];
			particle.addForce( gravity );
			particle.integrate( TIMESTEP_SQ );
		}

		// Start Constraints
		constraints = this.constraints;
		il = constraints.length;
		for ( i = 0; i < il; i ++ ) {
			constraint = constraints[ i ];
			this.satisifyConstraints( constraint[ 0 ], constraint[ 1 ], constraint[ 2 ] );
		}

		// Ball Constraints
		//ballPosition.z = - Math.sin( Date.now() / 600 ) * 90 ; //+ 40;
		//ballPosition.x = Math.cos( Date.now() / 400 ) * 70;
		//if ( sphere.visible ) {
		for ( particles = this.particles, i = 0, il = particles.length; i < il; i ++ ) {
			//console.log(ball.position);
			particle = particles[ i ];
			var pos = particle.position;
			//console.log(pos);
			this.diff.subVectors( pos, this.ball.position );
			originDiff = this.diff;

			var planeDiff = new THREE.Vector3();
			if (this.width == 0) {
				planeDiff = new THREE.Vector3(0, this.diff.y, this.diff.z);
			} else if (this.height == 0) {
				planeDiff = new THREE.Vector3(this.diff.x, 0, this.diff.z);
			} else {
				planeDiff = new THREE.Vector3(this.diff.x, this.diff.y, 0);
				//console.log(this.diff.x + " " + this.diff.y);
			}

			if ( this.diff.length() < ballSize || this.collided_before[i]){
						//(planeDiff.length() < ballSize && this.some_collided_before)) {
				// collided
				if (this.first_contact_point != undefined && this.ball.position.z < this.first_contact_point.z){
					this.diff.normalize().multiplyScalar( -1 * ballSize );
				} else {
					this.diff.normalize().multiplyScalar( ballSize );
				}
				pos.copy(this.ball.position).add(this.diff);
				if (!this.collided_before[i] && this.first_contact_sign == undefined) {
					this.first_contact_vel = this.ball.getLinearVelocity();
					if (this.width == 0)
						this.first_contact_sign = math.sign(this.ball.getLinearVelocity().x);
					else
						this.first_contact_sign = math.sign(this.ball.getLinearVelocity().z);
					this.first_contact_point[i] = new THREE.Vector3(pos.x, pos.y, pos.z);
				}
				this.collided_before[i] = true;
				this.some_collided_before = true;

				var vel;
				if (this.width == 0) {
					vel = this.ball.getLinearVelocity().x;
				} else {
					vel = this.ball.getLinearVelocity().z;
				}
				var sum = 0;
				for (var c=0; c<this.collided_before.length; c++) {
					if (this.collided_before[i])
					++sum;
				}

				//console.log(math.sign(vel) == this.first_contact_sign);
				if (math.sign(vel) == this.first_contact_sign || (math.sign(vel) != this.first_contact_sign && math.abs(vel) < 500)) {
						if (this.width == 0) {
							ball.applyCentralImpulse(new THREE.Vector3(-this.first_contact_sign * 50000/sum, 0, 0)); //originDiff.normalize().multiplyScalar(1000000/sum));
						} else
							ball.applyCentralImpulse(new THREE.Vector3(0, 0, 3000000/sum));//originDiff.normalize().multiplyScalar(1000000/sum));
							//originDiff.x *= -1;
							//ball.applyCentralImpulse(originDiff.normalize().multiplyScalar(1000000/sum));
					//ball.applyCentralImpulse(new THREE.Vector3(0,0,100000/sum));
				}

				var diffFirst;
				if (this.width == 0) {
					diffFirst = (pos.x - particle.original.x);// this.first_contact_point[i].x);
				} else if (this.height == 0) {
					diffFirst = (pos.y - particle.original.y);//this.first_contact_point[i].y);
				} else {
					diffFirst = (pos.z - particle.original.z);//this.first_contact_point[i].z);
				}
				var toll = 1000;
			//	if (pos.x > -3500 && pos.x < 3500 && pos.z > -1600
			//			|| pos.x > 3500+toll || pos.x < -3500-toll
			//			|| pos.z < -1600-toll) {
				if (math.abs(diffFirst) < 150 && math.sign(vel) != this.first_contact_sign) {
					this.collided_before[i] = false;
					if (sum == 1) {
						this.some_collided_before = false;
					}
					this.particles[i].position.copy(this.particles[i].original);
				}
				if (game.state == GameState.READY) {
					this.collided_before[i] = false;
					this.some_collided_before = false;
					this.first_contact_sign = undefined;
				}
			}
		}
		//}

		// Pin Constraints
		for ( i = 0, il = this.pins.length; i < il; i ++ ) {
			var xy = this.pins[ i ];
			var p = this.particles[ xy ];
			p.position.copy( p.original );
			p.previous.copy( p.original );
		}
	}

	this.clothFunction = clothFunction;
}

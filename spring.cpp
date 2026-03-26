#include "spring.hpp"

#include <algorithm>

bool isBallOnSpring(const RigidBody& ball, const Spring& spring) {
	auto pos = ball.getPosition();
	double r = ball.getRadius();
	double ballBottom = pos[1] - r;
	double springTop = spring.y + spring.height;

	return pos[0] >= spring.x && pos[0] <= spring.x + spring.width &&
		   ballBottom <= springTop && ballBottom >= spring.y;
}

void applySpringImpulse(RigidBody& ball, const Spring& spring) {
	auto pos = ball.getPosition();
	auto vel = ball.getVelocity();
	double r = ball.getRadius();
	double ballBottom = pos[1] - r;
	double springTop = spring.y + spring.height;

	double compression = springTop - ballBottom;

	if (compression > 0 && vel[1] < 0) {
		double bounceStrength = 1.0 + spring.springConstant * compression * 0.1;
		ball.setVelocityY(-vel[1] * std::min(bounceStrength, 2.0));
		ball.setPositionY(springTop + r);
	}
}


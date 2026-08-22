#include <SFML/Graphics.hpp>
#include <iostream>
#include "declarations.h"
#include <cmath>
#include <algorithm>


Friction::Friction(float mewk, float mews) : mewk(mewk), mews(mews) {}
float Friction::computeFriction(const Body& top) {
	if (top.getVelocity().x >= -.01 && top.getVelocity().x <= .01) {
		return 0.0f;
	}
	float normalForce = -(GRAVITY)*top.getMass();
	float sign = top.getVelocity().x > 0 ? 1.0f : -1.0f;
	return top.getVelocity().x >= 2 ? (top.getVelocity().x / top.getVelocity().x * sign * mewk * normalForce) : (top.getVelocity().x / top.getVelocity().x * sign * mews * normalForce);
}

Body::Body(float posX, float posY, float veloX, float veloY, unsigned int Mass)
	: position(posX, posY), velocity(veloX, veloY), mass(Mass), currentForces(0.0f, 0.0f)
{
}
void Body::applyForce(const sf::Vector2f& force) {
	currentForces += force;
}
void Body::updateAll(float dt) {
	acceleration = currentForces / (float)mass;
	velocity += acceleration * dt;
	position += velocity * dt;
	currentForces = sf::Vector2f(0.0f, 0.0f);
}
sf::Vector2f Body::getPosition() const {
	return position;
}
sf::Vector2f Body::getVelocity() const {
	return velocity;
}
sf::Vector2f Body::getAcceleration() const {
	return acceleration;
}
void Body::setPosition(sf::Vector2f position) {
	this->position = position;
}
void Body::setBodyRadius(float radius) {
	this->radius = radius;
}
unsigned int Body::getMass() const {
	return mass;
}
float Body::getBodyRadius() const {
	return radius;
}
void Body::setVelocityY(float veloY) {
	velocity.y = veloY;
}
void Body::setVelocityX(float veloX) {
	velocity.x = veloX;
}
std::string Body::computeEnergy() const {
	float kinetic = .5f * getMass() * (velocity.x * velocity.x + velocity.y * velocity.y);
	float potentialGravitational = getMass() * GRAVITY * (1080.0f - (position.y + getBodyRadius()));
	int energy = kinetic + potentialGravitational;
	return std::to_string(energy) + " joules";
}
void Body::conservationWalls() {
	if (position.y + radius >= 1080) {
		position.y = 1080 - radius; //move back to edge to avoid jittering or double bounce
		if (velocity.y > 0) {
			setVelocityY(-velocity.y);
		}
	}
	if (position.y - radius <= 0) {
		position.y = radius;
		if (velocity.y < 0) {
			setVelocityY(-velocity.y);
		}
	}
	if (position.x + radius >= 1920) {
		position.x = 1920 - radius;
		if (velocity.x > 0) {
			setVelocityX(-velocity.x);
		}
	}
	if (position.x - radius <= 0) {
		position.x = radius;
		if (velocity.x < 0) {
			setVelocityX(-velocity.x);
		}
	}
}

Circle::Circle(float posX, float posY, float veloX, float veloY, float radius, unsigned int Mass)
	: Body(posX, posY, veloX, veloY, Mass)
{
	circle.setRadius(radius);
	circle.setOrigin({ radius, radius });
	setBodyRadius(radius);
}
void Circle::refresh() {
	circle.setPosition(getPosition());
}
void Circle::circleCollisionCircle(Circle& body) {
	float deltaX = body.getPosition().x - this->getPosition().x;
	float deltaY = body.getPosition().y - this->getPosition().y;
	float distance = std::sqrt(deltaX * deltaX + deltaY * deltaY); //distance along parallel axis between centers  which points from this to body (assumed to be only axis 3rd law force acts)
	if (distance <= this->getBodyRadius() + body.getBodyRadius()) {
		float angleParallelRadians = std::atan2(deltaY, deltaX); //angle of distance line to horizontal
		float angleThisVelocityRadians = std::atan2(this->getVelocity().y, this->getVelocity().x) - angleParallelRadians;
		float angleOtherRadians = std::atan2(body.getVelocity().y, body.getVelocity().x) - angleParallelRadians;
		float thisVeloMag = std::sqrt((this->getVelocity().x * this->getVelocity().x + (this->getVelocity().y * this->getVelocity().y)));
		float bodyVeloMag = std::sqrt((body.getVelocity().x * body.getVelocity().x) + (body.getVelocity().y * body.getVelocity().y));
		float thisParallelVelocity = std::cos(angleThisVelocityRadians) * thisVeloMag; //velocity along parallel axis
		float bodyParallelVelocity = std::cos(angleOtherRadians) * bodyVeloMag; //same
		float thisPerpVelocity = std::sin(angleThisVelocityRadians) * thisVeloMag; //velocity along perpindicular axis to force
		float bodyPerpVelocity = std::sin(angleOtherRadians) * bodyVeloMag; //same
		float newThisParallelVelo = (this->getMass() * thisParallelVelocity - body.getMass() * thisParallelVelocity + 2 * body.getMass() * bodyParallelVelocity) / ((float)this->getMass() + body.getMass()); //calculated new velo from conservation equations
		float newOtherParallelVelo = (this->getMass() * thisParallelVelocity) / (float)body.getMass() + bodyParallelVelocity - ((this->getMass() * newThisParallelVelo) / (float)body.getMass());
		this->setPosition({ this->getPosition().x - (std::cos(angleParallelRadians) * ((this->getBodyRadius() + body.getBodyRadius() - distance) / 2.0f)), this->getPosition().y - (std::sin(angleParallelRadians) * ((this->getBodyRadius() + body.getBodyRadius() - distance) / 2.0f)) }); //move circle back to where it collided
		body.setPosition({ body.getPosition().x + (std::cos(angleParallelRadians) * ((this->getBodyRadius() + body.getBodyRadius() - distance) / 2.0f)), body.getPosition().y + (std::sin(angleParallelRadians) * ((this->getBodyRadius() + body.getBodyRadius() - distance) / 2.0f)) }); //same for other
		this->setVelocityX(std::cos(angleParallelRadians) * newThisParallelVelo + std::cos(angleParallelRadians + 1.570796) * thisPerpVelocity); //change velocity with new parallel component and unchanged perp component
		this->setVelocityY(std::sin(angleParallelRadians) * newThisParallelVelo + std::sin(angleParallelRadians + 1.570796) * thisPerpVelocity);
		body.setVelocityX(std::cos(angleParallelRadians) * newOtherParallelVelo + std::cos(angleParallelRadians + 1.570796) * bodyPerpVelocity);
		body.setVelocityY(std::sin(angleParallelRadians) * newOtherParallelVelo + std::sin(angleParallelRadians + 1.570796) * bodyPerpVelocity);
	}
}
void Circle::circleCollisionRectangle(Rectangle& other) {
	float leftWall = other.getPosition().x - other.getSize().x / 2.0;
	float rightWall = other.getPosition().x + other.getSize().x / 2.0;
	float topWall = other.getPosition().y - other.getSize().y / 2.0;
	float bottomWall = other.getPosition().y + other.getSize().y / 2.0;
	sf::Vector2f closestPt; //point on rectangle border that is closest to center's circle found through clamping
	if (this->getPosition().x >= leftWall && this->getPosition().x <= rightWall) {
		closestPt.x = this->getPosition().x;
	}
	else if (this->getPosition().x >= rightWall) {
		closestPt.x = rightWall;
	}
	else {
		closestPt.x = leftWall;
	}
	if (this->getPosition().y <= bottomWall && this->getPosition().y >= topWall) {
		closestPt.y = this->getPosition().y;
	}
	else if (this->getPosition().y >= bottomWall) {
		closestPt.y = bottomWall;
	}
	else {
		closestPt.y = topWall;
	}
	float deltaX = closestPt.x - this->getPosition().x;
	float deltaY = closestPt.y - this->getPosition().y;
	float distance = std::sqrt(deltaX * deltaX + deltaY * deltaY); //vector pointing from circle to rectangle
	if (distance <= getBodyRadius()) {
		Friction frict(.1, .13);
		this->applyForce({ frict.computeFriction(*this), 0 });
		float angleParallelRadians = std::atan2(deltaY, deltaX); // angle of shortest distance line to horizontal
		float angleThisVelocityRadians = std::atan2(this->getVelocity().y, this->getVelocity().x) - angleParallelRadians;
		float angleOtherRadians = std::atan2(other.getVelocity().y, other.getVelocity().x) - angleParallelRadians;
		float thisVeloMag = std::sqrt((this->getVelocity().x * this->getVelocity().x + (this->getVelocity().y * this->getVelocity().y)));
		float bodyVeloMag = std::sqrt((other.getVelocity().x * other.getVelocity().x) + (other.getVelocity().y * other.getVelocity().y));
		float thisParallelVelocity = std::cos(angleThisVelocityRadians) * thisVeloMag; // velocity along parallel axis
		float bodyParallelVelocity = std::cos(angleOtherRadians) * bodyVeloMag; // same
		float thisPerpVelocity = std::sin(angleThisVelocityRadians) * thisVeloMag; //velocity along perpindicular axis to angleParallelRadians
		float bodyPerpVelocity = std::sin(angleOtherRadians) * bodyVeloMag; //same
		float newThisParallelVelo = (this->getMass() * thisParallelVelocity - other.getMass() * thisParallelVelocity + 2 * other.getMass() * bodyParallelVelocity) / ((float)this->getMass() + other.getMass()); //calculated new velo from conservation equations
		float newOtherParallelVelo = (this->getMass() * thisParallelVelocity) / (float)other.getMass() + bodyParallelVelocity - ((this->getMass() * newThisParallelVelo) / (float)other.getMass());
		this->setPosition({ this->getPosition().x - std::cos(angleParallelRadians) * (getBodyRadius() - distance), this->getPosition().y - std::sin(angleParallelRadians) * (getBodyRadius() - distance) }); //move back to edge to avoid double collision
		this->setVelocityX(std::cos(angleParallelRadians) * newThisParallelVelo + std::cos(angleParallelRadians + 1.570796) * thisPerpVelocity); //change velocity with new parallel component and unchanged perp component
		this->setVelocityY(std::sin(angleParallelRadians) * newThisParallelVelo + std::sin(angleParallelRadians + 1.570796) * thisPerpVelocity);
		other.setVelocityX(std::cos(angleParallelRadians) * newOtherParallelVelo + std::cos(angleParallelRadians + 1.570796) * bodyPerpVelocity);
		other.setVelocityY(std::sin(angleParallelRadians) * newOtherParallelVelo + std::sin(angleParallelRadians + 1.570796) * bodyPerpVelocity);
	}
}
void Circle::draw(sf::RenderWindow& window) {
	window.draw(circle);

}
void Circle::changeColor(sf::Color color) {
	circle.setFillColor(color);
}

Rectangle::Rectangle(float posX, float posY, float veloX, float veloY, float length, float width, unsigned int Mass, sf::Angle angle, float omega)
	: Body(posX, posY, veloX, veloY, Mass), length(length), width(width), angle(angle), omega(omega)
{
	setSize(length, width);
	rectangle.setOrigin({ length / 2.0f, width / 2.0f });
}
void Rectangle::refresh() {
	rectangle.setPosition(getPosition());
	rectangle.setRotation(angle);
}
void Rectangle::setOmega(float omega) {
	this->omega = omega;
}
float Rectangle::getMoment() const {
	return 1 / 12.0f * getMass() * (length * length + width * width);
}
void Rectangle::draw(sf::RenderWindow& window) {
	window.draw(rectangle);
}
void Rectangle::applyTorque(float appliedTorque) {
	torque += appliedTorque;
}
void Rectangle::updateRotation(float dt) {
	angle += sf::radians(omega * dt);
}
void Rectangle::setSize(float length, float width) {
	rectangle.setSize({ length, width });
	setBodyRadius(std::sqrt(length * length + width * width) / 2.0f);
}
sf::Vector2f Rectangle::getSize() const {
	return { length, width };
}
float Rectangle::getOmega() const {
	return omega;
}
void Rectangle::conservationWalls() {
	if (getPosition().y - width / 2.0 <= 0 || getPosition().y + width / 2.0 >= 1080) {
		setVelocityY(-(getVelocity().y));
	}
	if (getPosition().x - length / 2.0 <= 0 || getPosition().x + length / 2.0 >= 1920) {
		setVelocityX(-(getVelocity().x));
	}
}
sf::Vector2f RotateAroundCenter(sf::Vector2f point, sf::Vector2f center, float radians) {
	point -= center;
	point = rotatePoint(point, radians);
	point += center;
	return point;
}
sf::Vector2f rotatePoint(sf::Vector2f point, float radians) {
	return { point.x * std::cos(radians) - point.y * std::sin(radians), point.x * std::sin(radians) + point.y * std::cos(radians) };
}
void Rectangle::rectangleCollisionWithRectangle(Rectangle& otherRect) {
	// separation axis theorem condition setup
	sf::Vector2f otherLeftTop = RotateAroundCenter({ otherRect.getPosition().x - otherRect.getSize().x / 2.0f, otherRect.getPosition().y - otherRect.getSize().y / 2.0f }, otherRect.getPosition(), otherRect.getAngle());
	sf::Vector2f otherRightTop = RotateAroundCenter({ otherRect.getPosition().x + otherRect.getSize().x / 2.0f, otherRect.getPosition().y - otherRect.getSize().y / 2.0f }, otherRect.getPosition(), otherRect.getAngle());
	sf::Vector2f otherRightBottom = RotateAroundCenter({ otherRect.getPosition().x + otherRect.getSize().x / 2.0f, otherRect.getPosition().y + otherRect.getSize().y / 2.0f }, otherRect.getPosition(), otherRect.getAngle());
	sf::Vector2f otherLeftBottom = RotateAroundCenter({ otherRect.getPosition().x - otherRect.getSize().x / 2.0f, otherRect.getPosition().y + otherRect.getSize().y / 2.0f }, otherRect.getPosition(), otherRect.getAngle());
	sf::Vector2f leftTop = RotateAroundCenter({ getPosition().x - getSize().x / 2.0f, getPosition().y - getSize().y / 2.0f }, getPosition(), getAngle());
	sf::Vector2f rightTop = RotateAroundCenter({ getPosition().x + getSize().x / 2.0f, getPosition().y - getSize().y / 2.0f }, getPosition(), getAngle());
	sf::Vector2f rightBottom = RotateAroundCenter({ getPosition().x + getSize().x / 2.0f, getPosition().y + getSize().y / 2.0f }, getPosition(), getAngle());
	sf::Vector2f leftBottom = RotateAroundCenter({ getPosition().x - getSize().x / 2.0f, getPosition().y + getSize().y / 2.0f }, getPosition(), getAngle());
	float axisAngleThisOne = std::atan2((double)(rightTop.y - leftTop.y), (double)(rightTop.x - leftTop.x)); //vector pointing across top face
	float axisAngleThisTwo = std::atan2(rightBottom.y - rightTop.y, rightBottom.x - rightTop.x); //vector pointing down rigt face
	float axisAngleOtherOne = std::atan2((double)(otherRightTop.y - otherLeftTop.y), (double)(otherRightTop.x - otherLeftTop.x)); //same for others
	float axisAngleOtherTwo = std::atan2(otherRightBottom.y - otherRightTop.y, otherRightBottom.x - otherRightTop.x);
	std::vector<float> axes = { axisAngleThisOne, axisAngleThisTwo, axisAngleOtherOne, axisAngleOtherTwo };
	float minOverlap = FLT_MAX;
	float currentOverlap;
	float minOverlapAxisAngle;

	for (int i = 0; i < 4; i++) { // check all axes for overlapping intervals
		float currentAngle = axes[i];
		float trTheta = 3.14159 / 2.0f - currentAngle - std::atan2(rightTop.x, rightTop.y); //angle bewteen 0,0 corner vector and projection axis
		float projectedTRPosition = std::sqrt(rightTop.x * rightTop.x + rightTop.y * rightTop.y) * std::cos(trTheta); //how much origin to point points in dir of projection axis (scalar projection)
		float tlTheta = 3.14159 / 2.0f - currentAngle - std::atan2(leftTop.x, leftTop.y); //same for different corners
		float projectedTLPosition = std::sqrt(leftTop.x * leftTop.x + leftTop.y * leftTop.y) * std::cos(tlTheta);
		float brTheta = 3.14159 / 2.0f - currentAngle - std::atan2(rightBottom.x, rightBottom.y);
		float projectedBRPosition = std::sqrt(rightBottom.x * rightBottom.x + rightBottom.y * rightBottom.y) * std::cos(brTheta);
		float blTheta = 3.14159 / 2.0f - currentAngle - std::atan2(leftBottom.x, leftBottom.y);
		float projectedBLPosition = std::sqrt(leftBottom.x * leftBottom.x + leftBottom.y * leftBottom.y) * std::cos(blTheta);
		float otherTrTheta = 3.14159 / 2.0f - currentAngle - std::atan2(otherRightTop.x, otherRightTop.y);
		float otherProjectedTRPosition = std::sqrt(otherRightTop.x * otherRightTop.x + otherRightTop.y * otherRightTop.y) * std::cos(otherTrTheta);
		float otherTlTheta = 3.14159 / 2.0f - currentAngle - std::atan2(otherLeftTop.x, otherLeftTop.y);
		float otherProjectedTLPosition = std::sqrt(otherLeftTop.x * otherLeftTop.x + otherLeftTop.y * otherLeftTop.y) * std::cos(otherTlTheta);
		float otherBrTheta = 3.14159 / 2.0f - currentAngle - std::atan2(otherRightBottom.x, otherRightBottom.y);
		float otherProjectedBRPosition = std::sqrt(otherRightBottom.x * otherRightBottom.x + otherRightBottom.y * otherRightBottom.y) * std::cos(otherBrTheta);
		float otherBlTheta = 3.14159 / 2.0f - currentAngle - std::atan2(otherLeftBottom.x, otherLeftBottom.y);
		float otherProjectedBLPosition = std::sqrt(otherLeftBottom.x * otherLeftBottom.x + otherLeftBottom.y * otherLeftBottom.y) * std::cos(otherBlTheta);
		float maxA = std::max({ projectedTRPosition, projectedTLPosition, projectedBRPosition, projectedBLPosition }); //maximum value along projection axis of interval A
		float minA = std::min({ projectedTRPosition, projectedTLPosition, projectedBRPosition, projectedBLPosition }); //same (min)
		float maxB = std::max({ otherProjectedTRPosition, otherProjectedTLPosition, otherProjectedBRPosition, otherProjectedBLPosition });
		float minB = std::min({ otherProjectedTRPosition, otherProjectedTLPosition, otherProjectedBRPosition, otherProjectedBLPosition });

		if (maxA < minB || maxB < minA) { //no overlap between intervals
			return; //quit check
		}
		currentOverlap = std::min(maxA, maxB) - std::max(minA, minB);
			if (currentOverlap < minOverlap) {
				minOverlap = currentOverlap;
				minOverlapAxisAngle = currentAngle;
				
			}
	}
	std::cout << "we collided!" << std::endl;
	//separate back and apply collision physics
	float collisionNormalDir = minOverlapAxisAngle; // aka projection axis containing projected corners
	sf::Vector2f n = { std::cos(collisionNormalDir), std::sin(collisionNormalDir) }; //unit vector (mag 1 dir of proj axis)
	sf::Vector2f centerToCenter = { //points from THIS to OTHER
	otherRect.getPosition().x - getPosition().x,
	otherRect.getPosition().y - getPosition().y
	};
	if (n.x * centerToCenter.x + n.y * centerToCenter.y < 0) { //n always points from this to other via dot product
		n = { -n.x, -n.y }; //rotate 180 deg
	}
	//find inner points (where overlap is) on projection interval and store which world coordinate point it is then average them to approximate where the collision occured
	float bestProjection = -FLT_MAX; // since this points to other, this is generally < other, so find maximum of this
	sf::Vector2f pointThis;
	for (sf::Vector2f p : {leftTop, rightTop, rightBottom, leftBottom}) {
		float projection = p.x * n.x + p.y * n.y;

		if (projection > bestProjection) {
			bestProjection = projection;
			pointThis = p; //back to world coordinate
		}
	}
	bestProjection = FLT_MAX; //vice versa for other
	sf::Vector2f pointOther;

	for (sf::Vector2f p : {otherLeftTop, otherRightTop,
		otherRightBottom, otherLeftBottom}) {
		float projection = p.x * n.x + p.y * n.y;

		if (projection < bestProjection) {
			bestProjection = projection;
			pointOther = p; //back to world
		}
	}
	sf::Vector2f collisionPoint = { (pointThis.x + pointOther.x) / 2.0f, (pointThis.y + pointOther.y) / 2.0f };
	sf::Vector2f distanceRThis = collisionPoint - getPosition();
	sf::Vector2f distanceROther = collisionPoint - otherRect.getPosition();
	sf::Vector2f V_This = getVelocity() + sf::Vector2f{distanceRThis.y * -getOmega(), distanceRThis.x * getOmega()}; //current velo + spinning contribution via cross prod (align r with rot velo then omega times r is v contribution)
	sf::Vector2f V_Other = otherRect.getVelocity() + sf::Vector2f{ distanceROther.y * -otherRect.getOmega(), distanceROther.x * otherRect.getOmega() }; //same

	float V_Relative = (V_This.x - V_Other.x) * n.x + (V_This.y - V_Other.y) * n.y; //velocity of this relative to other projected onto collision normal because thats where I'm solely assuming the collision impulse acts.
	if (V_Relative <= 0) return; //moving away from each other

	float c_this = distanceRThis.x * n.y - distanceRThis.y * n.x; //vector from this COM to collision point CROSS collision normal unit vector
	float c_other = distanceROther.x * n.y - distanceROther.y * n.x; // same for other 
	//(used in denominator of impulse calculation and derived from j cross n = I(omegaFinal - omegaInitial). After that cross is subbed in and relative velocity becomes a term, it repeats itself a couple times in the K term.
	float I_a = getMoment(); //rotational interia (formula found in getMoment() method
	float I_b = otherRect.getMoment();
	float K = (1.0f / getMass()) + (1.0f / otherRect.getMass()) //K is an ugly denominator that comes from factoring out j from the energy conservation equation. See README for math explanation.
	+ (c_this * c_this) / I_a+ (c_other * c_other) / I_b;
	float j = (-2 * V_Relative) / (K); //SCALE FACTOR OF THE IMPULSE with direction n (needed to solve for new linear and angular velocities)
	sf::Vector2f newV_This = getVelocity() + (j / getMass()) * n;
	sf::Vector2f newV_Other = otherRect.getVelocity() - (j / otherRect.getMass()) * n;
	float newOmega_This = getOmega() + j * c_this / I_a;
	float newOmega_Other = otherRect.getOmega() - j * c_other / I_b;
	setOmega(newOmega_This);
	otherRect.setOmega(newOmega_Other);
	setVelocityX(newV_This.x);
	setVelocityY(newV_This.y);
	otherRect.setVelocityX(newV_Other.x);
	otherRect.setVelocityY(newV_Other.y);
	sf::Vector2f correction = n * (minOverlap / 2.0f); //average to correct both equally (not the most flawless approach, but it looks realistic, especially because this is a difference of pixels.
	setPosition(getPosition() - correction); //this points to other, so subtracting from this makes this move away from other
	otherRect.setPosition(otherRect.getPosition() + correction); //vice versa

}
float Rectangle::getAngle() const {
	return angle.asRadians();

}

Gravity::Gravity() {}
sf::Vector2f Gravity::getForce(unsigned int Mass) const {
	return { 0.0f, GRAVITY * Mass };
}
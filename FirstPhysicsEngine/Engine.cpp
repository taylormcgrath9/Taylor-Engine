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

Rectangle::Rectangle(float posX, float posY, float veloX, float veloY, float length, float width, unsigned int Mass, sf::Angle angle, float omega, float alpha)
	: Body(posX, posY, veloX, veloY, Mass), length(length), width(width), angle(angle), omega(omega), alpha(alpha)
{
	setSize(length, width);
	rectangle.setOrigin({ length / 2.0f, width / 2.0f });
}
void Rectangle::refresh() {
	rectangle.setPosition(getPosition());
	rectangle.setRotation(angle);
}
void Rectangle::draw(sf::RenderWindow& window) {
	window.draw(rectangle);
}
void Rectangle::applyTorque(float appliedTorque) {
	torque += appliedTorque;
}
void Rectangle::updateRotation(float dt) {
	alpha = torque / momentInertia;
	omega += alpha * dt;
	angle += sf::degrees(omega * dt);
	torque = 0;
}
void Rectangle::setSize(float length, float width) {
	rectangle.setSize({ length, width });
	setBodyRadius(std::sqrt(length * length + width * width) / 2.0f);
}
sf::Vector2f Rectangle::getSize() const {
	return { length, width };
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
	// separation axis theorem condition
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
		
		if (std::max({ projectedTRPosition, projectedTLPosition, projectedBRPosition, projectedBLPosition }) < std::min({ otherProjectedTRPosition, otherProjectedTLPosition, otherProjectedBRPosition, otherProjectedBLPosition })
			|| std::max({ otherProjectedTRPosition, otherProjectedTLPosition, otherProjectedBRPosition, otherProjectedBLPosition }) < std::min({ projectedTRPosition, projectedTLPosition, projectedBRPosition, projectedBLPosition })) {
		//no overlap between intervals
			return;
		}
	}
	//separate back and apply collision physics


}
float Rectangle::getAngle() const {
	return angle.asRadians();

}

Gravity::Gravity() {}
sf::Vector2f Gravity::getForce(unsigned int Mass) const {
	return { 0.0f, GRAVITY * Mass };
}
#include <SFML/Graphics.hpp>
#include <iostream>
#include "declarations.h"
#include <cmath>


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

Rectangle::Rectangle(float posX, float posY, float veloX, float veloY, float length, float width, unsigned int Mass)
	: Body(posX, posY, veloX, veloY, Mass), length(length), width(width)
{
	setSize(length, width);
	rectangle.setOrigin({ length / 2.0f, width / 2.0f });
}
void Rectangle::refresh() {
	rectangle.setPosition(getPosition());
}
void Rectangle::draw(sf::RenderWindow& window) {
	window.draw(rectangle);
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
void Rectangle::rotateRectangle(float degrees) {
	angle = degrees;
	rectangle.rotate(sf::degrees(degrees));
}

Gravity::Gravity() {}
sf::Vector2f Gravity::getForce(unsigned int Mass) const {
	return { 0.0f, GRAVITY * Mass };
}
#pragma once
#include <iostream>
#include <SFML/Graphics.hpp>
#include <tuple>
constexpr float GRAVITY = 7.0f;
class Circle;
class Rectangle;
class Body {
private:
	sf::Vector2f position;
	sf::Vector2f velocity;
	sf::Vector2f acceleration;
	unsigned int mass;
	sf::Vector2f currentForces;
	sf::Angle angle;
	float omega;
public:
	Body(float posX, float posY, float veloX, float veloY, unsigned int Mass, sf::Angle angle, float omega);
	void applyForce(const sf::Vector2f& force);
	void updateAll(float dt);
	sf::Vector2f getPosition() const;
	sf::Vector2f getVelocity() const;
	sf::Vector2f getAcceleration() const;
	void setPosition(sf::Vector2f position);
	unsigned int getMass() const;
	void setVelocityY(float veloY);
	void setVelocityX(float veloX);
	void updateRotation(float dt);
	std::tuple<int, std::string> computeEnergy() const;
	sf::Angle getAngle() const;
	float getOmega() const;
	void setOmega(float omega);
	virtual void conservationWalls() = 0;
	virtual void refresh() = 0;
	virtual float getMoment() const = 0;
	virtual void changeColor(sf::Color color) = 0;
};

class Circle : public Body {
private:
	sf::CircleShape circle;
	float radius;
public:
	Circle(float posX, float posY, float veloX, float veloY, float radius, unsigned int Mass, sf::Angle angle, float omega);
	void refresh() override;
	void circleCollisionCircle(Circle& other);
	void conservationWalls() override;
	void circleCollisionRectangle(Rectangle& other);
	void draw(sf::RenderWindow& window);
	float getMoment() const override;
	void changeColor(sf::Color color) override;
};

class Rectangle : public Body {
private:
	sf::RectangleShape rectangle;
	float length;
	float width;
	float momentInertia;
public:
	Rectangle(float posX, float posY, float veloX, float veloY, float length, float width, unsigned int Mass, sf::Angle angle, float omega);
	void refresh() override;
	void draw(sf::RenderWindow& window);
	void setSize(float length, float width);
	void applyTorque(float appliedTorque);
	sf::Vector2f getSize() const;
	float getMoment() const override;
	void conservationWalls() override;
	void rectangleCollisionWithRectangle(Rectangle& otherRect);
	void changeColor(sf::Color color) override;
};

class Gravity 
{
public:
	Gravity();
	sf::Vector2f getGravity(unsigned int Mass) const;
};

//non member functions
sf::Vector2f rotatePoint(sf::Vector2f point, float radians);
sf::Vector2f RotateAroundCenter(sf::Vector2f point, sf::Vector2f center, float radians);
std::string computeSystemEnergy(const std::vector<Body*>& bodies);
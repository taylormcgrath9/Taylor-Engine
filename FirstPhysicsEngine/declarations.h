#pragma once
#include <iostream>
#include <SFML/Graphics.hpp>
class Circle;
class Rectangle;


class Body {
private:
	sf::Vector2f position;
	sf::Vector2f velocity;
	sf::Vector2f acceleration;
	unsigned int mass;
	sf::Vector2f currentForces;
	float radius;

public:
	Body(float posX, float posY, float veloX, float veloY, unsigned int Mass);
	
	void applyForce(const sf::Vector2f& force);
	void updateAll(float dt);
	sf::Vector2f getPosition() const;
	sf::Vector2f getVelocity() const;
	sf::Vector2f getAcceleration() const;
	void setPosition(sf::Vector2f position);
	float getBodyRadius() const;
	void setBodyRadius(float radius);
	unsigned int getMass() const;
	void setVelocityY(float veloY);
	void setVelocityX(float veloX);
	virtual void conservationWalls();
	virtual void refresh() = 0;
	
};

class Circle : public Body {
private:
	sf::CircleShape circle;

public:
	Circle(float posX, float posY, float veloX, float veloY, float radius, unsigned int Mass);
	void refresh() override;
	void circleCollisionCircle(Circle& other);
	void circleCollisionRectangle(Rectangle& other);
	void draw(sf::RenderWindow& window);
	
};

class Rectangle : public Body {
private:
	sf::RectangleShape rectangle;
	float length;
	float width;
public:
	Rectangle(float posX, float posY, float veloX, float veloY, float length, float width, unsigned int Mass);
	void refresh() override;
	void draw(sf::RenderWindow& window);
	void setSize(float length, float width);
	sf::Vector2f getSize() const;
	void conservationWalls() override;

};

class Force {
private:
	sf::Vector2f forceVector;
public:
	virtual sf::Vector2f getForce(unsigned int Mass) const {
		return forceVector;
	}


};

class Gravity : public Force
{
private:
	float g;
public:
	Gravity(float g);
	sf::Vector2f getForce(unsigned int Mass) const override;
};

class Friction : public Force {
private:
	float mewk;
	float mews;

public:
	Friction(float mewk, float mews);
	float computeFriction(const Body& other, float g);



};
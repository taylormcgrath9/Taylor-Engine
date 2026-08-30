#include <iostream>
#include <SFML/Graphics.hpp>
#include "declarations.h"


int main()
{
	Circle ball(0, 250, 0, 0, 60, 50, sf::radians(0), 0);
	ball.changeColor(sf::Color::Blue);
	sf::Font font;
	font.openFromFile("arial.ttf");
	sf::Angle Angle = sf::radians(0);
	Rectangle rect(300, 300, -30, 0, 300, 30, 45, Angle, -.5);
	Rectangle rect2(800, 500, 10, 0, 60, 60, 20, Angle,1);
	rect2.changeColor(sf::Color::Green);
	std::vector<Body*> systemBodies = {&ball, &rect, &rect2};
	Gravity gravity;
	sf::RenderWindow window(sf::VideoMode({ 1920, 1080 }), "Taylor Engine");
	
	while (window.isOpen()) {
		while (const std::optional event = window.pollEvent())
		{

			if (event->is<sf::Event::Closed>())
				window.close();
		}
		ball.applyForce(gravity.getGravity(ball.getMass()));
		sf::Text text(font, std::get<1>(ball.computeEnergy()), 40);
		sf::Text text2(font, std::get<1>(rect.computeEnergy()), 40);
		sf::Text text3(font, std::get<1>(rect2.computeEnergy()), 40);
		sf::Text totalEnergy(font, computeSystemEnergy(systemBodies), 40);
		text.setFillColor(sf::Color::Blue);
		text2.setFillColor(sf::Color::White);
		text3.setFillColor(sf::Color::Green);
		totalEnergy.setFillColor(sf::Color::Yellow);
		text.setPosition({ 25, 20 });
		text2.setPosition({ 300, 20 });
		text3.setPosition({ 575, 20 });
		totalEnergy.setPosition({ 850, 20 });
		ball.updateRotation(1/(60.0f));
		ball.updateAll(1 / (60.0));
		rect.applyForce(gravity.getGravity(rect.getMass()));
		rect.updateRotation(1 / 60.0);
		rect.updateAll(1 / 60.0);
		rect2.applyForce(gravity.getGravity(rect2.getMass()));
		rect2.updateRotation(1 / 60.0);
		rect2.updateAll(1 / 60.0);
		ball.conservationWalls();
		rect.conservationWalls();	
		rect2.conservationWalls();
		rect.rectangleCollisionWithRectangle(rect2);
		ball.circleCollisionRectangle(rect);
		ball.circleCollisionRectangle(rect2);
		ball.refresh();
		rect.refresh();
		rect2.refresh();
		window.clear();
		ball.draw(window);
		rect.draw(window);
		rect2.draw(window);
		window.draw(text);
		window.draw(text2);
		window.draw(text3);
		window.draw(totalEnergy);
		window.display();
	}
	return 0;
}
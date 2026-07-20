#include <iostream>
#include <SFML/Graphics.hpp>
#include "declarations.h"


int main()
{
	Circle ball(1020, 100, -50, -30, 40, 30);
	Circle ball2(935, 955, 20, 0, 85, 90);
	ball.changeColor(sf::Color::Blue);
	sf::Font font;
	font.openFromFile("arial.ttf");
	Rectangle rect(960, 1030, 0, 0, 1920, 30, 100000000000);
	Gravity gravity;
	sf::RenderWindow window(sf::VideoMode({ 1920, 1080 }), "Taylor Engine");

	while (window.isOpen()) {
		while (const std::optional event = window.pollEvent())
		{

			if (event->is<sf::Event::Closed>())
				window.close();
		}
		ball.applyForce(gravity.getForce(ball.getMass()));
		ball2.applyForce(gravity.getForce(ball2.getMass()));
		sf::Text text(font, ball.computeEnergy(), 40);
		sf::Text text2(font, ball2.computeEnergy(), 40);
		text.setFillColor(sf::Color::Blue);
		text.setPosition({ 25, 50 });
		text2.setPosition({ 300, 50 });
		ball.updateAll(1 / (60.0));
		ball2.updateAll(1 / 60.0);
		rect.updateAll(1 / 60.0);
		ball2.updateAll(1 / 60.0);
		ball.conservationWalls();
		ball2.conservationWalls();
		rect.conservationWalls();
		ball.circleCollisionRectangle(rect);
		ball.circleCollisionCircle(ball2);
		ball2.circleCollisionRectangle(rect);
		ball.refresh();
		rect.refresh();
		ball2.refresh();
		window.clear();
		ball.draw(window);
		rect.draw(window);
		ball2.draw(window);
		window.draw(text);
		window.draw(text2);
		window.display();
	}
	return 0;
}
#include <iostream>
#include <SFML/Graphics.hpp>
#include "declarations.h"


int main()
{
    Circle ball(200, 740, 63,0,50, 60);
    Rectangle rect(960, 800, 0, 0, 1920, 30, 10000000);
    Gravity gravity(7.0f);


    sf::RenderWindow window(sf::VideoMode({ 1920, 1080 }), "Taylor Engine");
  
 
    while (window.isOpen())
    {
        
        while (const std::optional event = window.pollEvent())
        {
         
            if (event->is<sf::Event::Closed>())
                window.close();
        }
        ball.applyForce(gravity.getForce(ball.getMass()));
       //rect.applyForce(gravity.getForce(rect.getMass()));
      
  
      
   
        ball.updateAll(1 / (60.0));
     
        rect.updateAll(1 / 60.0);
        ball.conservationWalls();
       
        rect.conservationWalls();
        
    ball.circleCollisionRectangle(rect);
    
        ball.refresh();
        rect.refresh();

    
        window.clear();
        ball.draw(window);
        rect.draw(window);
       
      
        window.display();
 
    }
    return 0;
}
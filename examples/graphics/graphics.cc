// BoB robotics includes
#include "robots/simulated_tank.h"

// Third-party includes
#include "third_party/units.h"

// SFML
#include <SFML/Graphics.hpp>

int
main()
{
    constexpr int WindowWidth = 800;
    constexpr int WindowHeight = 600;

    // Create window; make it non-resizable
    sf::RenderWindow window(sf::VideoMode(WindowWidth, WindowHeight),
                            "BoB robotics",
                            sf::Style::Titlebar | sf::Style::Close);

    window.setVerticalSyncEnabled(true);

    // Put red cross at origin
    constexpr float lineWidth = 3.f;
    constexpr float lineLength = 20.f;
    sf::RectangleShape hline({ lineLength, lineWidth });
    hline.setFillColor(sf::Color::Red);
    hline.setPosition({ (WindowWidth - lineLength) / 2.f, (WindowHeight - lineWidth) / 2.f });
    sf::RectangleShape vline({ lineWidth, lineLength });
    vline.setFillColor(sf::Color::Red);
    vline.setPosition({ (WindowWidth - lineWidth) / 2.f, (WindowHeight - lineLength) / 2.f });

    // run the program as long as the window is open
    while (window.isOpen()) {
        // check all the window's events that were triggered since the last iteration of the loop
        sf::Event event;
        while (window.pollEvent(event)) {
            // "close requested" event: we close the window
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }

        // Set background colour
        window.clear(sf::Color::White);

        // Draw cross at origin
        window.draw(hline);
        window.draw(vline);

        // Swap buffers
        window.display();
    }
}
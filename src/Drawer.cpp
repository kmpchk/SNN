#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <string>
#include "Drawer.h"
#include <vector>
#include <iostream>
#include "spdlog/spdlog.h"

Drawer::Drawer(float a, float b, float b2, float c, int chars)
{
    radius = a;
    distx = b;
    disty = b2;
    margin = c;
    char_size = chars;
    this->texture;
    if (!font.loadFromFile("res/calibri.ttf")) {
        std::cout << "font not found";
    }
    for (int i=0; i < levels; i++) {
        numbers.emplace_back(1);
    }
}

void Drawer::add_node(int level, std::string text)
{
    int number = numbers[level];
    numbers[level]++;
    int posx = margin + (distx + radius * 2) * (number - 1);
    int posy = margin + disty * (level - 1) + radius * 2 * level;

    Pos pos;
    pos.posx = posx;
    pos.posy = size_y - posy;
    pos.name = text;

    nodes.emplace_back(pos);

    if (posx > size_x - radius){
        size_x = size_x + radius * 4;
    }

    if (posy > size_y - radius){
        size_y = size_y + radius * 4;
        for (int i=0; i < nodes.size(); i++) {
            nodes[i].posy += radius * 4;
        }
    }
}

void Drawer::init_texture(){
    texture.create(size_x, size_y);
}

void Drawer::draw()
{
    for (int i=0; i < nodes.size(); i++) {
        int posx = nodes[i].posx;
        int posy = nodes[i].posy;
        std::string text = nodes[i].name;
        sf::CircleShape shape(radius);
        shape.setFillColor(sf::Color::White);
        shape.setPosition(posx, posy);
        texture.draw(shape);

        sf::Text label;
        label.setFont(font);
        label.setString(text);
        label.setColor(sf::Color::Black);
        label.setCharacterSize(char_size);
        label.setScale(1.f, -1.f);
        label.setPosition(posx + 2, posy + radius + char_size / 2);
        texture.draw(label);
    }
}

void Drawer::connect_nodes(std::string name1, std::string name2)
{
    Pos posa;
    Pos posb;
    for (int i=0; i < nodes.size(); i++) {
        if (nodes[i].name == name1){
            posa = nodes[i];
        }
        else if (nodes[i].name == name2)
        {
            posb = nodes[i];
        }
        //spdlog::debug("[draw x = {0}, y = {1}, name = {2}", nodes[i].posx, nodes[i].posy, nodes[i].name);
    }
    
    //spdlog::debug("[a x = {0}, y = {1}, name = {2}", posa.posx, posa.posy, posa.name);
    //spdlog::debug("[b x = {0}, y = {1}, name = {2}", posb.posx, posb.posy, posb.name);
    
    sf::VertexArray lines(sf::Lines, 2);

    lines[0].position = sf::Vector2f(posa.posx + radius, posa.posy + radius);
    lines[1].position = sf::Vector2f(posb.posx + radius, posb.posy + radius);
    lines[0].color = sf::Color::Red;
    lines[1].color = sf::Color::Blue;
    texture.draw(lines);
}

void Drawer::save(std::string name)
{
    texture.getTexture().copyToImage().saveToFile(name);
}
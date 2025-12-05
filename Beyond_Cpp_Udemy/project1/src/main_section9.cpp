#include <iostream>

using namespace std;

int main()
{

    enum class MenuItem
    {
        p, // Print a number
        a, // Add a number
        m, // mean number
        s, // smallest numbers
        l, // Largest numbers
        q  // Quit
    };

    char selection{};
    while (true)
    {
        cout << "Enter your choice: p, a, m, s, l, q: ";
        cin >> selection;
        // selection = std::tolower(selection);
        cout << selection << " was selected" << endl;
        cout << static_cast<MenuItem>(selection) << endl;

        switch (static_cast<MenuItem>(selection))
        {
        case MenuItem::p:
            cout << "Print number" << endl;
            break;
        case MenuItem::a:
            cout << "Add number" << endl;
            break;
        case MenuItem::m:
            cout << "Mean number" << endl;
            break;
        case MenuItem::s:
            cout << "Smallest number" << endl;
            break;
        case MenuItem::l:
            cout << "Largest number" << endl;
            break;
        case MenuItem::q:
            cout << "Quit" << endl;
            return 0;
        default:
            cout << "Unknown selection, please try again" << endl;
        }
    }
    return 0;
}

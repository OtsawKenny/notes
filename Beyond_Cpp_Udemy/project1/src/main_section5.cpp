#include <iostream>

extern int x;

using namespace std;

int main() {
    // int favourite_number;
    // cout << "Enter your favourite number between 1 and 100: ";
    // cin >> favourite_number;
    // cout << "Amazing! " << favourite_number << " is my favourite number too!" << endl;

    // std::cout <<x;

    int num1;
    double num2;

    cout << "Enter a first digit: ";
    cin >> num1;

    cout << "Enter a second digit: ";
    cin >> num2;

    cout << "Your numbers are: " << num1 << " and " << num2 << endl;
    return 0;
}


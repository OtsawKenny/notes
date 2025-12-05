#include <iostream>

using namespace std;

int main()
{
    // char vowels[] {'a','e','i','o','u'};
    // cout<<"The first vowel is "<<vowels[0]<<endl;
    // cout<<"The second vowel is "<<vowels[1]<<endl;
    // cin>>vowels[4];

    int movie_rating[3][4]{
        {0, 1, 3, 5},
        {0, 1, 3, 4},
        {0, 1, 3, 5}};

    cout << "Movie rating for row 1 column 3 is: " << movie_rating[1][3] << endl;
    return 0;
}

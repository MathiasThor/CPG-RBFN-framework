/*****************************************************************************
 *  Copyright (C) 2012 by Timo Nachstedt                                     *
 *                                                                           *
 *  This program is free software: you can redistribute it and/or modify     *
 *  it under the terms of the GNU General Public License as published by     *
 *  the Free Software Foundation, either version 3 of the License, or        *
 *  (at your option) any later version.                                      *
 *                                                                           *
 *  This program is distributed in the hope that it will be useful,          *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            *
 *  GNU General Public License for more details.                             *
 *                                                                           *
 *  You should have received a copy of the GNU General Public License        *
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.    *
 *                                                                           *
 ****************************************************************************/

#ifndef INTERPOLATOR2D_H_
#define INTERPOLATOR2D_H_

#include <vector>

/**
 * Class to represent interpolatable data
 *
 * This class can read a two row data table representing a y(x) relationship
 * from a file and afterwards answer queries to get y values for a specific
 * x and the other way round. If the exact value is not contained in the data
 * table a linear interpolation between the two nearest data points is
 * calculated.
 *
 * At the moment the data has be sorted from low to high x values
 */
class Interpolator2d
{
public:
    /**
     * The constructor
     */
    Interpolator2d();

    /**
     * Loads a data table from a file
     *
     * This method tries to load the data from the file with the given filename.
     * The file must contain at least two data columns where the first column
     * contains the x values and the second row contains the y values. Comments
     * can be added by starting the line with the # sign.
     *
     * @param filename name of the data file
     */
    void load(const char* filename);

    /**
     * Returns the interpolated x value for a given y
     *
     * This method returns the exact or interpolated x value belonging to the
     * given y value.
     *
     * @param y the y value
     * @return exact or interpolated x value
     */
    double x(const double& y) const;

    /**
     * Returns the interpolated y value for a give x
     *
     * This method returns the exact or interpolated y value belonging to the
     * give x value.
     *
     * @param x the x value
     * @param exact or interpolated y value
     */
    double y(const double& x) const;
private:

    /** Struct representing one data table entry */
    struct tableEntry {
        double x;
        double y;
    };

    /** Contains all data table entries */
    std::vector<tableEntry> table;
    /** minimum x value of the currently loaded data table */
    double minX;
    /** maximum x value of the currently loaded data table */
    double maxX;
    /** minimum y value of the currently loaded data table */
    double minY;
    /** maximum y value of the currently loaded data table */
    double maxY;
    /** number of entries of the currently loaded data table */
    int    N;
};


#endif /* INTERPOLATOR2D_H_ */

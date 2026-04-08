/**
 * Calculates the percentage price change over a given number of days.
 * @param {object} data - The historical price data object.
 * @param {number} days - The number of days to look back.
 * @returns {number} - The percentage change.
 */
export const calculatePriceChange = (data, days) => {
  if (data?.historical && data.historical.length > days) {
    const previousPrice = data.historical[days].close;
    const currentPrice = data.historical[0].close;
    return ((currentPrice - previousPrice) / previousPrice) * 100;
  }
  return 0;
};

/**
 * Calculates the moving average over a given number of days.
 * @param {object} data - The historical price data object.
 * @param {number} days - The number of days for the moving average window.
 * @returns {number} - The moving average price.
 */
export const calculateMovingAverage = (data, days) => {
  if (data?.historical && data.historical.length >= days) {
    const prices = data.historical.slice(0, days).map((entry) => entry.close);
    const sum = prices.reduce((a, b) => a + b, 0);
    return sum / days;
  }
  return 0;
};

/**
 * Calculates the custom I4B Index score.
 * @param {object} company - The company object with e, s, and g scores.
 * @param {object} financialRatios - The collection of financial ratios for all selected companies.
 * @returns {number} - The calculated I4B Index score.
 */
export const calculateIndex = (company, financialRatios) => {
  const esgScore = company.e + company.s + company.g;
  const ratios = financialRatios[company.ticker];

  if (ratios && esgScore) {
    const margin = ratios["Net Profit Margin (in %)"] || 0;
    // Avoid division by zero
    return esgScore > 0 ? margin / esgScore : 0;
  }
  return 0;
};

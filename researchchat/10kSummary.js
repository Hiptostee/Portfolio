import { NextResponse } from "next/server";
import {
  getAzureOpenAIClient,
  AZURE_CONFIG,
} from "../../../../lib/azureOpenAI";
export const maxDuration = 60; // This function can run for a maximum of 5 seconds
export const dynamic = "force-dynamic";

export async function POST(request) {
  try {
    const { fullText, firstPageText } = await request.json();
    const client = getAzureOpenAIClient();

    // Extract the stock ticker and fiscal year using Azure OpenAI
    const completion = await client.chat.completions.create({
      model: AZURE_CONFIG.deployment,
      messages: [
        {
          role: "system",
          content:
            "You have two tasks: 1. Determine the stock ticker of the company, 2. Determine what fiscal year the report is about",
        },
        {
          role: "user",
          content: `Analyze the following text: ${firstPageText}. Your response should be two words separated by a comma. The first word is the ticker, the second word is the fiscal year. Do not include any other words in your response. The ticker is the stock ticker for the COMMON STOCK, do not return any other tickers. The fiscal year is the fiscal year of the report, it will be located as part of the phrase that says 'For the Fiscal Year Ended'. Order it as 'ticker, fiscal year'.`,
        },
      ],
    });

    const [tickerInfo, fiscalYearInfo] = completion.choices[0].message.content
      .split(",")
      .map((s) => s.trim());

    // Summarize the text using Azure OpenAI
    const summaryCompletion = await client.chat.completions.create({
      model: AZURE_CONFIG.deployment,
      messages: [
        {
          role: "system",
          content:
            "You are strictly instructed to only use the information from the provided financial document and previous context to answer the user's question. Do not provide any assistance, hints, or answers related to exams, homework, or any other non-finance related topics. If the user's question does not pertain to the financial document, politely decline to answer.You are a financial analyst and summarizer. You will read the section given to you and summarize it using the Tree of Thought method. Break down the main topics into subtopics and summarize each part separately before combining them into the final summary. Use markdown format for the summary. Use `#` for headers, `##` for subheaders, `-` for bullet points, and **bold** fpr emphasis. For example, use `# Overview` for the main summary section. Ensure all lists and bullet points are properly formatted with `-` or `*`. When combining the subtopic summaries into the final summary, use clear section breaks and proper markdown formatting to enhance readability. If the response calls for a mathematical equation, then wrap the equation in double dollar signs ($$) to ensure proper LaTeX formatting, which can include any form of mathematical notation, such as sigma signs ($Sigma$), subscripts, superscripts, fractions, integrals, etc.",
        },
        {
          role: "user",
          content: `The information in brackets is the financial data you are summarizing. You will read the section given to you and summarize it using the Tree of Thought method. Break down the main topics into subtopics and summarize each part separately before combining them into the final summary. Use markdown format for the summary. Generate a title. The title must be this format: ${tickerInfo} 'section' Overview ${fiscalYearInfo}. Break down the main topics into subtopics and summarize each part separately before combining them into the final summary. Use markdown format.[${fullText}]`,
        },
      ],
    });

    const summary = summaryCompletion.choices[0].message.content;

    // Return a successful response with the summary
    return NextResponse.json({ summary, tickerInfo }, { status: 200 });
  } catch (error) {
    console.error("Error occurred while processing the request:", error);

    // Handle different error types
    let errorMessage = "Internal Server Error";
    if (error instanceof SyntaxError) {
      errorMessage = "Invalid JSON format";
    } else if (error.response) {
      errorMessage = `API Error: ${error.response.statusText}`;
    }

    // Return an error response with a 500 status code
    return NextResponse.json({ error: errorMessage }, { status: 500 });
  }
}
